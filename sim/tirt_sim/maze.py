"""迷宮世界模型 —— 模擬器裡唯一「知道真實世界長怎樣」的東西。

設計取捨(為什麼是 2D 佔據柵格,而不是 Gazebo 的 3D 物理):

  比賽場地是「20cm 高的白色不透光牆 + 平地」,而規則只准用光達。也就是說
  這台車能感知到的世界**本來就是 2D 的** —— 光達在固定高度切一刀,切出來的
  就是這裡的柵格。用 3D 物理引擎去算重力、摩擦、輪子接觸點,算完再切一刀
  投影回 2D,對「學會這套系統怎麼運作」沒有任何額外資訊,卻要付出:裝
  Gazebo、寫 SDF、調 mecanum plugin、以及在沒有 GPU 的 VM 裡跑 3D 算繪。

  所以這裡直接在 2D 做,而且刻意做成**假物理**(見 fake_base_node.py)。
  真實的打滑/PID/電池垂降等到實機再處理 —— 那些本來就不可能模擬得準。

座標系:世界原點 = 迷宮左下角外緣,x 向右、y 向上,和 REP-103 一致。
        存成地圖時這個原點就是 map.yaml 的 origin,所以模擬座標和 Nav2
        的 map 座標可以完全對齊(見 make_map.py)。
"""

import math

import numpy as np
import yaml


class MazeWorld:
    """一個迷宮場地:牆的佔據柵格 + 起點/終點/中繼點等語意資訊。"""

    def __init__(self, cfg):
        self.name = cfg.get('name', 'unnamed')
        self.cell = float(cfg.get('cell', 0.44))        # 每格淨寬(規則:44cm)
        self.wall = float(cfg.get('wall', 0.02))        # 牆厚
        self.res = float(cfg.get('resolution', 0.02))   # 模擬柵格解析度
        ox, oy = cfg.get('origin', [0.0, 0.0])
        self.origin = (float(ox), float(oy))

        self.pitch = self.cell + self.wall              # 格心間距(規則:46cm)

        h_walls, v_walls, self.rows, self.cols = _parse_ascii(cfg['maze'])
        self.width = self.cols * self.pitch + self.wall
        self.height = self.rows * self.pitch + self.wall

        self.grid = self._rasterize(h_walls, v_walls, cfg.get('obstacles', []))
        self.ny, self.nx = self.grid.shape

        # 語意標記。座標可以寫成 [x, y](公尺)或 "cell r,c"(格號,左上為 0,0),
        # 後者在照著主辦單位發的路徑圖畫迷宮時方便得多。
        self.start = self._resolve_pose(cfg.get('start', [0.5, 0.5, 0.0]))
        self.goal = self._resolve_pose(cfg.get('goal')) if cfg.get('goal') else None
        self.checkpoint = (self._resolve_pose(cfg.get('checkpoint'))
                           if cfg.get('checkpoint') else None)

    # ---------- 建構 ----------

    def _rasterize(self, h_walls, v_walls, obstacles):
        """把牆的拓樸(哪兩格之間有牆)畫成細柵格。True = 被佔據。"""
        nx = int(round(self.width / self.res))
        ny = int(round(self.height / self.res))
        grid = np.zeros((ny, nx), dtype=bool)

        half = self.wall / 2.0

        def fill(x0, y0, x1, y1):
            """把世界座標的一個矩形塗成佔據。"""
            i0 = max(0, int(math.floor((x0 - self.origin[0]) / self.res)))
            i1 = min(nx, int(math.ceil((x1 - self.origin[0]) / self.res)))
            j0 = max(0, int(math.floor((y0 - self.origin[1]) / self.res)))
            j1 = min(ny, int(math.ceil((y1 - self.origin[1]) / self.res)))
            if i1 > i0 and j1 > j0:
                grid[j0:j1, i0:i1] = True

        # 格線位置。第 i 條縱線 x = origin_x + wall/2 + i*pitch。
        def gx(i):
            return self.origin[0] + half + i * self.pitch

        def gy(j):
            return self.origin[1] + half + j * self.pitch

        # 橫牆:h_walls[j][i] = 第 j 條橫線上、第 i 格的位置有牆
        # (j 從下往上數 0..rows,i 從左往右 0..cols-1)
        for j in range(self.rows + 1):
            for i in range(self.cols):
                if h_walls[j][i]:
                    # 兩端各多延伸 half,讓轉角是實心的 —— 否則光達會從
                    # 牆角的針孔漏出去,掃出真實世界不存在的細長光束。
                    fill(gx(i) - half, gy(j) - half,
                         gx(i + 1) + half, gy(j) + half)

        # 直牆:v_walls[j][i] = 第 i 條縱線上、第 j 格的位置有牆
        for j in range(self.rows):
            for i in range(self.cols + 1):
                if v_walls[j][i]:
                    fill(gx(i) - half, gy(j) - half,
                         gx(i) + half, gy(j + 1) + half)

        # 障礙物(規則:25x25x20cm,正式比賽前才由裁判放進障礙區)。
        # 寫成世界座標的矩形 [x, y, w, h],x/y 是**中心**。
        for ob in obstacles:
            x, y, w, h = (list(ob) + [0.25, 0.25])[:4]
            fill(x - w / 2, y - h / 2, x + w / 2, y + h / 2)

        return grid

    def _resolve_pose(self, spec):
        """接受 [x, y] / [x, y, yaw_deg] / 'cell r,c' / 'cell r,c,yaw_deg'。"""
        if isinstance(spec, str):
            body = spec.split(None, 1)[1] if spec.lower().startswith('cell') else spec
            parts = [float(p) for p in body.replace(',', ' ').split()]
            r, c = parts[0], parts[1]
            yaw = parts[2] if len(parts) > 2 else 0.0
            x, y = self.cell_center(int(r), int(c))
            return (x, y, math.radians(yaw))
        vals = [float(v) for v in spec]
        if len(vals) == 2:
            vals.append(0.0)
        return (vals[0], vals[1], math.radians(vals[2]))

    def cell_center(self, row, col):
        """格號(左上角為 0,0,和 ASCII 圖上看到的一樣)-> 世界座標格心。"""
        j = self.rows - 1 - row          # ASCII 由上往下,世界 y 由下往上
        x = self.origin[0] + self.wall / 2 + (col + 0.5) * self.pitch
        y = self.origin[1] + self.wall / 2 + (j + 0.5) * self.pitch
        return (x, y)

    # ---------- 查詢 ----------

    def is_occupied(self, x, y):
        i = int((x - self.origin[0]) / self.res)
        j = int((y - self.origin[1]) / self.res)
        if i < 0 or j < 0 or i >= self.nx or j >= self.ny:
            return True                  # 界外一律當成牆
        return bool(self.grid[j, i])

    def raycast(self, x, y, yaw, angles, range_max, step=None):
        """從 (x, y) 朝 yaw+angles 各射一條光線,回傳撞牆距離(numpy array)。

        用「固定步長取樣 + numpy 向量化」而不是逐條 DDA:beam 數 x 步數的
        查表在 numpy 裡是一次矩陣操作,純 Python 迴圈則會慢到追不上 10Hz。
        步長取解析度的一半,確保不會從一格牆中間穿過去。

        沒撞到任何東西時回傳 inf —— LaserScan 的規範是超出量程要填
        range_max 以外的值,交給呼叫端處理,這裡不做假設。
        """
        step = step or self.res * 0.5
        # 光線最遠只需要走到場地對角線;C1 的 12m 量程在 4m 的迷宮裡是純浪費,
        # 少算 60% 的取樣點對每秒 10 次的掃描很有感。
        reach = min(range_max, math.hypot(self.width, self.height) + self.res)
        n_steps = max(2, int(reach / step))

        dists = np.arange(1, n_steps + 1, dtype=np.float32) * step     # (S,)
        th = (yaw + angles).astype(np.float32)                          # (B,)

        # (B, S) 的取樣點座標
        xs = x + np.cos(th)[:, None] * dists[None, :]
        ys = y + np.sin(th)[:, None] * dists[None, :]

        ii = ((xs - self.origin[0]) / self.res).astype(np.int32)
        jj = ((ys - self.origin[1]) / self.res).astype(np.int32)

        out = (ii < 0) | (jj < 0) | (ii >= self.nx) | (jj >= self.ny)
        np.clip(ii, 0, self.nx - 1, out=ii)
        np.clip(jj, 0, self.ny - 1, out=jj)

        hit = self.grid[jj, ii] | out           # 界外視同撞牆

        # 每條光線第一個 True 的位置。argmax 對全 False 會回 0,所以要另外
        # 用 any() 把「整條都沒撞到」挑出來,否則會全部變成距離 0。
        first = hit.argmax(axis=1)
        any_hit = hit.any(axis=1)
        rng = (first + 1).astype(np.float32) * step
        return np.where(any_hit, rng, np.inf)

    # ---------- 匯出 ----------

    def to_occupancy_png(self, out_res):
        """重新取樣成 Nav2 map_server 要的解析度,回傳 (H, W) uint8 影像。

        任一細柵格被佔據,整個粗格就算佔據(保守)。牆只有 2cm,而地圖
        解析度是 5cm —— 取多數決的話牆會整片消失,地圖就變成一片空地。
        """
        scale = out_res / self.res
        ny = max(1, int(math.floor(self.ny / scale)))
        nx = max(1, int(math.floor(self.nx / scale)))
        img = np.zeros((ny, nx), dtype=np.uint8)
        for j in range(ny):
            j0, j1 = int(j * scale), max(int(j * scale) + 1, int((j + 1) * scale))
            for i in range(nx):
                i0, i1 = int(i * scale), max(int(i * scale) + 1, int((i + 1) * scale))
                img[j, i] = 0 if self.grid[j0:j1, i0:i1].any() else 254
        return img


def _parse_ascii(text):
    """解析標準迷宮 ASCII 圖。

        +--+--+--+          橫線列(索引 0,2,4...):'-' = 有橫牆
        |        |          格子列(索引 1,3,5...):'|' = 有直牆
        +  +--+  +
        |  |  |  |
        +--+--+--+

    每格佔 3 個字元寬,所以 W 格的圖寬是 3W+1、H 列的圖高是 2H+1。
    回傳 (h_walls, v_walls, rows, cols),兩個 walls 的索引都已經轉成
    「y 由下往上」,和世界座標一致。
    """
    lines = [ln.rstrip('\n') for ln in text.strip('\n').split('\n')]
    lines = [ln for ln in lines if ln.strip()]
    if len(lines) < 3 or len(lines) % 2 == 0:
        raise ValueError(f'迷宮圖列數應為奇數且 >=3,實際 {len(lines)} 列')

    rows = (len(lines) - 1) // 2
    cols = (max(len(ln) for ln in lines) - 1) // 3
    lines = [ln.ljust(3 * cols + 1) for ln in lines]

    # h_walls[j][i]:第 j 條橫線(0=最下)、第 i 格上方有沒有牆
    h_walls = [[False] * cols for _ in range(rows + 1)]
    v_walls = [[False] * (cols + 1) for _ in range(rows)]

    for r in range(rows + 1):
        line = lines[2 * r]
        j = rows - r                      # ASCII 由上往下 -> 世界由下往上
        for i in range(cols):
            h_walls[j][i] = line[3 * i + 1] == '-'

    for r in range(rows):
        line = lines[2 * r + 1]
        j = rows - 1 - r
        for i in range(cols + 1):
            v_walls[j][i] = line[3 * i] == '|'

    return h_walls, v_walls, rows, cols


def load_world(path):
    with open(path, 'r', encoding='utf-8') as f:
        return MazeWorld(yaml.safe_load(f))
