
 本機實測(剛剛跑起整套 bringup):

   ┌──────────┬─────────────────────────────────────────────────────────────────────┬─────────┐
     │   項目   │                              源頭實測                               │  判定   │
       ├──────────┼─────────────────────────────────────────────────────────────────────┼─────────┤
         │ /scan    │ 穩定 10Hz,frame laser_frame,時間戳正常(100ms 一筆)                  │ ✅ 乾淨 │
           ├──────────┼─────────────────────────────────────────────────────────────────────┼─────────┤
             │ /odom    │ 20Hz 超穩(min 0.047 / max 0.052s,std 0.001s)                        │ ✅ 乾淨 │
               ├──────────┼─────────────────────────────────────────────────────────────────────┼─────────┤
                 │ /tf      │ ~60Hz 穩定                                                          │ ✅ 乾淨 │
                   ├──────────┼─────────────────────────────────────────────────────────────────────┼─────────┤
                     │ TF 鏈    │ laser_frame→base_link→odom 完整,PC slam 已連線且有 /map             │ ✅ 完整 │
                       ├──────────┼─────────────────────────────────────────────────────────────────────┼─────────┤
                         │ 網路設定 │ wlan0 IP 對、省電 off、DDS 白名單只放 LAN+localhost(沒放 tailscale) │ ✅ 正確 │
                           └──────────┴─────────────────────────────────────────────────────────────────────┴─────────┘

                             所以 queue is full 不是設定錯、也不是源頭爛 —— 是 WiFi 傳輸的延遲抖動造成的,跟你自己在 config 第 20 行記的「WiFi 突發傳輸 /scan 停頓達
                               0.47s」完全吻合。

                                 機制:Pi 端 odom TF 是穩穩 20Hz,但 WiFi 一卡頓,scan 到了 PC、對應那一刻的 odom→base_link TF 卻還沒到,slam 的 message filter 就把 scan
                                   排進佇列等;WiFi 一路抖,佇列排到爆 → 丟 scan → queue is full。transform_timeout 已經拉到 1.0(上限級)只是治標。

                                     這是網路鏈路品質問題,不是 ROS 設定問題。 這也正是 networkplan.md 想解決的事。

                                       ---
                                         方向的取捨是你的決定,我先問清楚要往哪走(其中「減少過網路 TF 量」我現在就能改):

                                         ● User answered Claude's questions:
                                           ⎿  · queue is full 已確定是 WiFi 抖動造成(Pi 源頭完全正常)。你想先往哪個方向處理? → 推進 networkplan.md 專用熱點

                                           ● 好選擇 —— 治本。我先讀 networkplan.md 看你已經寫到哪、還缺什麼,再把它推成一個可以直接照做的方案。

                                             Read 1 file (ctrl+o to expand)

                                             ● 讀完了。這份計畫已經把「為什麼要自帶熱點」講清楚,卡在最後一步:還沒實際寫進 Pi 的網路設定。要正確寫入,我得先搞清楚 Pi
                                               現在的網路是用哪一套在管(NetworkManager 還是 netplan+networkd)—— 之前你那條 nmcli ... device show wlan0 回空,暗示 wlan0 可能不是
                                                 NetworkManager 在管。我查一下:

                                                 ● Bash(echo "=== NetworkManager 狀態 ==="; systemctl is-active NetworkManager 2>/dev/null; echo "=== networkd 狀態 ==="; systemctl is-active
                                                       systemd-networkd 2>/dev/null
                                                             e…)
                                                               ⎿  === NetworkManager 狀態 ===
                                                                    active
                                                                         === networkd 狀態 ===
                                                                              … +14 lines (ctrl+o to expand)

                                                                              ● 查清楚了,關鍵一點:wlan0 不是 NetworkManager 在管,是 netplan → systemd-networkd 在管(NM 顯示 wlan0 unmanaged,而 netplan 產生了
                                                                                /run/netplan/wpa-wlan0.conf 跑 wpa_supplicant)。所以熱點要寫進 netplan,不是 nmcli。

                                                                                  好消息:netplan 的 wifi 可以在同一個 wlan0 底下列多組 access-points 並帶 priority,wpa_supplicant
                                                                                    會自動連上當下掃得到的那一個。這正好能做到「宿舍家用 WiFi + 手機熱點 + 筆電熱點」三組並存、自動切換。而 DDS 那端 setup_dds.sh
                                                                                      本來就會自動偵測當下 wlan0 的 IP 重新渲染白名單,換網段不用手改 —— 所以唯一缺的就是把兩組熱點寫進 netplan。

                                                                                        我打算寫成這個結構(保留你現在的家用 WiFi,不動它):

                                                                                          network:
                                                                                              version: 2
                                                                                                  wifis:
                                                                                                        wlan0:
                                                                                                                dhcp4: true
                                                                                                                        access-points:
                                                                                                                                  "你家WiFi的SSID":            # 保留,priority 最低
                                                                                                                                              password: "..."
                                                                                                                                                          priority: 1
                                                                                                                                                                    "手機熱點SSID":              # 主用,有 4G/Tailscale
                                                                                                                                                                                password: "..."
                                                                                                                                                                                            priority: 10
                                                                                                                                                                                                      "筆電熱點SSID":              # 備援,保證無 AP isolation
                                                                                                                                                                                                                  password: "..."
                                                                                                                                                                                                                              priority: 5

                                                                                                                                                                                                                                套用時我會用 sudo netplan try(斷線會 60 秒自動回滾),所以就算設錯也不會把你現在的連線弄死。

                                                                                                                                                                                                                                  ---
                                                                                                                                                                                                                                    要動手,我需要你兩樣東西(這是上一輪就卡住的地方):

                                                                                                                                                                                                                                      1. 現有 netplan 內容(root 才讀得到,我讀不了),請在輸入框打:
                                                                                                                                                                                                                                        ! sudo cat /etc/netplan/50-cloud-init.yaml

                                                                                                                                                                                                                                          2. 兩組熱點的 SSID + 密碼:
                                                                                                                                                                                                                                            - 手機熱點:你手機開熱點後的 SSID 和密碼(主用)
                                                                                                                                                                                                                                              - 筆電熱點:你想幫筆電熱點取的固定 SSID + 密碼(備援,以後不改)—— 這組筆電那邊我也會給你一行 nmcli device wifi hotspot ... 指令去開

                                                                                                                                                                                                                                                拿到這兩樣,我就把設定寫好、netplan try 套用,再把「到新場地的開機 SOP」補進 雙機RViz連線.md。

                                                                                                                                                                                                                                                  同時我先把這次的診斷結論存進記憶(免得下次又從「SLAM 壞掉」重查):

                                                                                                                                                                                                                                                    Recalled 1 memory, wrote 2 memories (ctrl+o to expand)

                                                                                                                                                                                                                                                    ● 診斷結論已存進記憶,下次不用重查。

                                                                                                                                                                                                                                                      目前狀態總結:

                                                                                                                                                                                                                                                        - ✅ 確認 queue is full 是 WiFi 傳輸抖動造成,Pi 源頭一切正常(實測 /scan 10Hz、/odom 20Hz 超穩、TF 鏈完整、PC slam 有連上)
                                                                                                                                                                                                                                                          - ✅ 你選了治本方向:自帶專用熱點
                                                                                                                                                                                                                                                            - ✅ 查清楚 wlan0 是 netplan/networkd 在管,所以熱點寫進 netplan、用 netplan try 安全套用
                                                                                                                                                                                                                                                              - ⏸️  卡在等你提供:①! sudo cat /etc/netplan/50-cloud-init.yaml 的內容 ②手機熱點 SSID+密碼、③筆電熱點你要取的 SSID+密碼

                                                                                                                                                                                                                                                                把這三樣給我,我就直接把 netplan 設定寫好套用,並補上場地開機 SOP。你先把手機熱點打開、! sudo cat 那行貼上來就行。
