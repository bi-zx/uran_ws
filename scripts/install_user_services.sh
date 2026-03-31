#!/bin/bash
# 安装用户级 systemd 服务的脚本

set -e

echo "安装 URAN 用户服务..."

# 创建用户 systemd 目录
mkdir -p /usr/lib/systemd/user

# 复制服务文件
sudo cp ./uran_core.user.service /usr/lib/systemd/user/uran_core.service
sudo cp ./uran_media.user.service /usr/lib/systemd/user/uran_media.service
sudo cp ./uran_move.user.service /usr/lib/systemd/user/uran_move.service

# 重新加载用户服务
systemctl --user daemon-reload

# 启用服务（开机自启）
systemctl --user enable uran_core.service
systemctl --user enable uran_media.service
systemctl --user enable uran_move.service

# 启动服务
systemctl --user start uran_core.service
systemctl --user start uran_media.service
systemctl --user start uran_move.service

echo "用户服务安装完成！"
echo ""
echo "常用命令："
echo "  查看状态: systemctl --user status uran_core.service"
echo "  查看日志: journalctl --user -u uran_core.service -f"
echo "  重启服务: systemctl --user restart uran_core.service"
echo "  停止服务: systemctl --user stop uran_core.service"
echo "  禁用服务: systemctl --user disable uran_core.service"
