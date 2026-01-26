"""Network utilities for ROS2 Viser - IP address detection and QR code display."""

import os
import socket
import struct
import fcntl
import logging
import qrcode
from typing import Optional, Tuple

logger = logging.getLogger(__name__)


def _get_interface_ip(ifname: str) -> Optional[str]:
    """
    获取指定网卡的 IPv4 地址
    
    Args:
        ifname: 网卡名称
        
    Returns:
        IP地址字符串，如果获取失败返回None
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ifreq = struct.pack('256s', ifname[:15].encode('utf-8'))
        res = fcntl.ioctl(sock.fileno(), 0x8915, ifreq)  # SIOCGIFADDR
        ip_addr = socket.inet_ntoa(res[20:24])
        sock.close()
        return ip_addr
    except OSError:
        return None


def get_local_ip() -> Tuple[str, Optional[str], bool]:
    """
    获取本机局域网 IP 地址，优先级：Wi-Fi > 有线网卡 > 默认路由
    
    Returns:
        (ip_addr, interface_name, is_wifi) 元组
    """
    wifi_prefixes = ["wlp0", "wl", "wlan", "wifi"]
    ethernet_prefixes = ["eth", "eno", "enp", "ens", "enx"]
    wifi_candidates = []
    ethernet_candidates = []

    # 检查环境变量
    env_iface = os.environ.get("WIFI_INTERFACE")
    if env_iface:
        wifi_candidates.append(env_iface.strip())

    try:
        all_ifaces = os.listdir("/sys/class/net")
    except FileNotFoundError:
        all_ifaces = []

    # 收集Wi-Fi候选网卡
    for prefix in wifi_prefixes:
        for iface in all_ifaces:
            if iface.startswith(prefix) and iface not in wifi_candidates:
                wifi_candidates.append(iface)

    # 优先使用 Wi-Fi 网卡
    for iface in wifi_candidates:
        ip_addr = _get_interface_ip(iface)
        if ip_addr and not ip_addr.startswith("127."):
            logger.debug(f'使用 Wi-Fi 网卡 {iface}，IP: {ip_addr}')
            return ip_addr, iface, True

    # 如果Wi-Fi不可用，查找有线网卡
    for prefix in ethernet_prefixes:
        for iface in all_ifaces:
            if iface.startswith(prefix) and iface not in ethernet_candidates:
                ethernet_candidates.append(iface)

    # 使用有线网卡
    for iface in ethernet_candidates:
        ip_addr = _get_interface_ip(iface)
        if ip_addr and not ip_addr.startswith("127."):
            logger.debug(f'使用有线网卡 {iface}，IP: {ip_addr}')
            return ip_addr, iface, False

    # 最后回退到默认路由
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        local_ip = sock.getsockname()[0]
        sock.close()
        
        # 尝试找到对应的网卡名称
        default_iface = None
        for iface in all_ifaces:
            if iface == "lo":
                continue
            ip = _get_interface_ip(iface)
            if ip == local_ip:
                default_iface = iface
                break
        
        if default_iface:
            logger.debug(f'使用默认路由网卡 {default_iface}，IP: {local_ip}')
            return local_ip, default_iface, False
        else:
            logger.debug(f'使用默认路由 IP: {local_ip} (未找到对应网卡名称)')
            return local_ip, None, False
    except Exception:
        logger.warning('⚠️ 无法获取本机 IP，使用占位地址 192.168.x.x')
        return "192.168.x.x", None, False


def get_viser_server_port(server, default_port: int = 8080) -> int:
    """
    从 viser 服务器对象获取实际监听的端口号
    
    Args:
        server: viser.ViserServer 实例
        default_port: 如果无法获取端口时的默认端口
        
    Returns:
        服务器端口号
    """
    import time
    # 等待服务器完全启动
    time.sleep(0.5)
    
    # 从 _websock_server._port 获取端口
    try:
        if hasattr(server, '_websock_server'):
            websock_server = server._websock_server
            if hasattr(websock_server, '_port'):
                port = websock_server._port
                if isinstance(port, int) and 1024 <= port <= 65535:
                    return port
    except:
        pass
    
    logger.warning(f"⚠️ 无法获取服务器端口，使用默认端口: {default_port}")
    return default_port


def display_server_info(server_url: str, local_ip: str, interface_name: Optional[str], 
                       is_wifi: bool, server_port: int) -> None:
    """
    显示服务器连接信息
    
    Args:
        server_url: 服务器URL
        local_ip: 本地IP地址
        interface_name: 网络接口名称
        is_wifi: 是否为Wi-Fi
        server_port: 服务器端口
    """
    logger.info("=" * 80)
    logger.info("🌐 Viser 服务器已启动")
    logger.info(f"📍 本地 IP 地址: {local_ip}")
    if interface_name:
        logger.info(f"🌐 网络接口: {interface_name} {'(Wi-Fi)' if is_wifi else ''}")
    logger.info(f"🔌 服务器端口: {server_port}")
    logger.info(f"🔗 访问 URL: {server_url}")
    logger.info("=" * 80)
    
    # 显示二维码
    display_server_qrcode(server_url)


def display_server_qrcode(server_url: str) -> None:
    """
    在终端显示服务器URL的二维码（ASCII格式）
    
    Args:
        server_url: 服务器URL
    """

    try:
        # 生成二维码
        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_L,
            box_size=1,  # 终端显示使用较小的box_size
            border=2,
        )
        qr.add_data(server_url)
        qr.make(fit=True)

        # 获取ASCII字符串
        qr.print_ascii(invert=False)
    except Exception as e:
        logger.warning(f'⚠️ 生成终端二维码失败: {e}')

