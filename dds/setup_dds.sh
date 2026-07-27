# 挑選並套用本機的 Fast DDS profile。SOURCE 它(不要執行),讓環境變數留在你的 shell:
#
#     source /home/lego/2026_TIRT/dds/setup_dds.sh
#
# 兩台機器都是同一行 —— 不再需要 DDS_SERVER=<pi_ip>,也不再需要每次換場地重新渲染。
# 角色由「本機持有哪個固定別名 IP」自動判斷:
#     持有 10.77.0.2 -> Pi  -> dds/fastdds_pi.xml
#     持有 10.77.0.1 -> PC  -> dds/fastdds_pc.xml
# 要強制指定時:TIRT_ROLE=pi|pc source dds/setup_dds.sh
#
# 固定別名是靠 net/install_pi_network.sh(Pi)/ net/install_pc_alias.sh(PC)裝上去的。
# 別名不存在時本腳本會**大聲報錯並且不設定任何東西** —— 因為靜默失聯比壞掉更難查:
# 舊版會自動退回本機當下的 DHCP IP,結果是節點看似正常啟動卻永遠找不到對方。

_dds_dir="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"

if [ -r "$_dds_dir/../net/tirt_net.conf" ]; then
    . "$_dds_dir/../net/tirt_net.conf"
else
    echo "setup_dds: 找不到 net/tirt_net.conf" >&2
    TIRT_PI_IP=10.77.0.2; TIRT_PC_IP=10.77.0.1
fi

_dds_has_ip() { ip -4 -o addr show scope global 2>/dev/null | grep -q " $1/"; }

if [ -n "${TIRT_ROLE:-}" ]; then
    _dds_role="$TIRT_ROLE"
elif _dds_has_ip "$TIRT_PI_IP"; then
    _dds_role=pi
elif _dds_has_ip "$TIRT_PC_IP"; then
    _dds_role=pc
else
    _dds_role=""
fi

if [ -z "$_dds_role" ]; then
    # 一定要清掉,不能只是「不設定」:舊版是把範本 sed 成 /run/.../fastdds_active.xml
    # 再 export,而 ~/.bashrc 會 source 本檔 —— 於是環境裡可能還留著上一版的值。
    # 各啟動腳本的守衛都是檢查這個變數是否為空,殘值會讓它們誤判成「DDS 設好了」,
    # 然後帶著一個指向不存在 / 位址已過期的 profile 啟動,症狀是無聲失聯。
    unset FASTRTPS_DEFAULT_PROFILES_FILE
    rm -f "${XDG_RUNTIME_DIR:-/tmp}/fastdds_active.xml" 2>/dev/null
    echo "setup_dds: 錯誤 —— 本機沒有固定別名 IP($TIRT_PI_IP 或 $TIRT_PC_IP),DDS 沒有設定。" >&2
    echo "setup_dds:   Pi  請跑:sudo ./net/install_pi_network.sh" >&2
    echo "setup_dds:   筆電請跑:sudo ./net/install_pc_alias.sh" >&2
    echo "setup_dds:   目前位址:$(ip -4 -o addr show scope global | awk '{print $2"="$4}' | tr '\n' ' ')" >&2
else
    _dds_profile="$_dds_dir/fastdds_${_dds_role}.xml"
    if [ ! -r "$_dds_profile" ]; then
        echo "setup_dds: 找不到 profile:$_dds_profile" >&2
    elif [ "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" = "$_dds_profile" ]; then
        # 已經是對的了就安靜跳過。~/.bashrc 有 source 本檔,而 pi/robot_tmux.sh 與
        # gcs.sh 的視窗初始化也會再 source 一次 —— 沒有這個保護的話每開一個視窗
        # 都會印兩遍相同訊息,真正重要的錯誤反而被淹掉。
        :
    else
        export FASTRTPS_DEFAULT_PROFILES_FILE="$_dds_profile"
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export ROS_LOCALHOST_ONLY=0
        echo "setup_dds: role=$_dds_role  server=${TIRT_PI_IP}:${TIRT_DDS_PORT:-11811}  profile=$_dds_profile"

        # ros2 CLI 的 daemon 會沿用「它第一次啟動時」的 discovery 設定,所以在這個
        # profile 生效前就跑起來的 daemon 會完全忽略它(症狀:topic list 空空如也 /
        # echo 說 could not determine type)。停掉它,下一道 ros2 指令會用新設定重生。
        if command -v ros2 >/dev/null 2>&1; then ros2 daemon stop >/dev/null 2>&1 || true; fi
    fi
fi

unset _dds_dir _dds_role _dds_profile
unset -f _dds_has_ip 2>/dev/null || true
