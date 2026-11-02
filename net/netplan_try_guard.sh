# 由 install_pi_network.sh / install_pi_eth.sh source。
#
# 為什麼需要:netplan 0.107.1(Ubuntu 22.04)的 `netplan try` 在 120 秒沒按 Enter、
# 自動回滾時,會把 /run/systemd/network/10-netplan-*.network 重寫成 root:root 0600。
# systemd-networkd 是以 systemd-network 使用者身分執行,讀不到 → 日誌出現
#   Failed to open configuration file '.../10-netplan-wlan0.network': Permission denied
# 結果 wifi 雖然連上 AP,卻「沒有任何 IP」(DHCP 和 10.77.0.2 都沒了),Pi 完全連不到,
# 只能重開機。2026-11-02 跑 install_pi_eth.sh 時實際踩到:回滾這個「安全網」本身把網路弄斷。
# 正常開機產生的檔案是 root:systemd-network 0640,不受影響;只有 try 回滾那條路會壞。
#
# 這裡在腳本結束(不論成功、回滾或出錯)時檢查權限,壞了就修好並叫 networkd 重讀。

_tirt_fix_networkd_perms() {
    local f fixed=0
    for f in /run/systemd/network/10-netplan-*; do
        [ -e "$f" ] || continue
        if [ "$(stat -c %G "$f")" != "systemd-network" ]; then
            chgrp systemd-network "$f" && chmod 640 "$f" && fixed=1
        fi
    done
    if [ "$fixed" = 1 ]; then
        echo "[guard] netplan 回滾把設定檔權限弄壞了(networkd 讀不到),已修復並重新載入。" >&2
        networkctl reload || true
    fi
}
trap _tirt_fix_networkd_perms EXIT
