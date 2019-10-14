# lidar_3x3lab
## raspberry piの設定
### screenのインストール
- バックグラウンドで実行する用
- sudo apt install -y screen
### 自動起動設定
1. /etc/rc.localに/home/pi/kc111_lidar/start.shと記載
2. sudo raspi-configを実行
3. 3.Boot Optionsを選択
4. B2 Wait for Network at Bookをyes
 
