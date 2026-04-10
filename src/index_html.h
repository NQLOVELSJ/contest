// index_html.h
#ifndef INDEX_HTML_H
#define INDEX_HTML_H

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>机器人声音警报</title>
    <style>
        body { font-family: sans-serif; display: flex; flex-direction: column; justify-content: center; align-items: center; height: 100vh; margin: 0; transition: background-color 0.3s; }
        .container { text-align: center; padding: 20px; background: rgba(0,0,0,0.1); border-radius: 20px; }
        #distance { font-size: 6rem; font-weight: bold; }
        #status { font-size: 2rem; margin-bottom: 20px; text-transform: uppercase; }
        .info { margin-top: 20px; color: #555; }
    </style>
</head>
<body id="mainBody">
    <div class="container">
        <h1>🤖 机器人状态</h1>
        <div id="status">⚪️ 离线</div>
        <div id="distance">--</div>
        <div style="font-size: 1.5rem;">厘米</div>
        <div class="info" id="detailInfo">等待连接...</div>
        <!-- 新增一个按钮，用于首次启用声音 -->
        <button id="enableAudioBtn" style="margin-top: 30px; padding: 12px 24px; font-size: 18px; border: none; border-radius: 8px; background-color: #007bff; color: white; cursor: pointer;">
            🔔 启动声音通知
        </button>
    </div>

    <script>
        const statusDiv = document.getElementById('status');
        const distanceDiv = document.getElementById('distance');
        const mainBody = document.getElementById('mainBody');
        const detailInfo = document.getElementById('detailInfo');
        const enableAudioBtn = document.getElementById('enableAudioBtn');
        
        let ws;
        let audioContext;
        let audioEnabled = false; // 一个标志，确保只在用户点击后启用音频

        // 1. 初始化音频上下文（但不在加载时创建）
        function initAudioContext() {
            if (!audioContext) {
                audioContext = new (window.AudioContext || window.webkitAudioContext)();
            }
            // 注意：AudioContext 一开始是挂起(suspended)状态，需要在用户交互后通过 .resume() 启动
            if (audioContext.state === 'suspended') {
                audioContext.resume();
            }
            audioEnabled = true;
            enableAudioBtn.textContent = '✅ 声音已启动';
            enableAudioBtn.disabled = true;
            enableAudioBtn.style.backgroundColor = '#28a745';
            console.log('Audio context started.');
        }

        // 按钮点击事件：启用声音
        enableAudioBtn.addEventListener('click', initAudioContext);

        // 2. 播放障碍物警报声的函数 (使用Web Audio API合成声音[reference:6])
        function playObstacleAlert() {
            if (!audioEnabled || !audioContext) {
                console.warn('音频未启用或上下文未初始化');
                return;
            }
            
            // 确保音频上下文是运行状态
            if (audioContext.state === 'suspended') {
                audioContext.resume();
            }

            const now = audioContext.currentTime;
            
            // 创建振荡器 (产生声音)
            const oscillator = audioContext.createOscillator();
            oscillator.type = 'sawtooth'; // 锯齿波，听起来像警报
            oscillator.frequency.setValueAtTime(880, now); // 设置频率为880Hz
            
            // 创建增益节点 (控制音量)
            const gainNode = audioContext.createGain();
            gainNode.gain.setValueAtTime(0, now); // 开始时音量为0
            gainNode.gain.linearRampToValueAtTime(0.5, now + 0.05); // 0.05秒内音量线性增加到0.5
            gainNode.gain.linearRampToValueAtTime(0.3, now + 0.2);
            gainNode.gain.linearRampToValueAtTime(0, now + 0.5); // 0.5秒内音量线性减小到0
            
            // 连接节点: 振荡器 -> 增益节点 -> 输出设备
            oscillator.connect(gainNode);
            gainNode.connect(audioContext.destination);
            
            // 播放
            oscillator.start();
            oscillator.stop(now + 0.5); // 0.5秒后停止
        }

        // 3. WebSocket 连接和处理
        function connectWebSocket() {
            const wsUrl = `ws://${window.location.hostname}/ws`;
            ws = new WebSocket(wsUrl);

            ws.onopen = function() {
                console.log('WebSocket 连接成功');
                statusDiv.textContent = '🟢 已连接';
                statusDiv.style.color = 'green';
            };

            ws.onmessage = function(event) {
                try {
                    const data = JSON.parse(event.data);
                    updateUI(data); // 调用之前的UI更新函数
                    
                    // 如果收到障碍物警报，则播放声音
                    if (data.type === 'alert' && data.message === 'obstacle_detected') {
                        playObstacleAlert();
                    } else if (data.status === 'Obstacle') {
                        // 兼容之前的逻辑：如果状态是障碍物，也播放声音
                        playObstacleAlert();
                    }
                } catch (e) {
                    console.error('JSON 解析错误:', e);
                }
            };

            ws.onerror = function(error) {
                console.error('WebSocket 错误:', error);
                statusDiv.textContent = '⚠️ 连接错误';
            };

            ws.onclose = function() {
                console.log('WebSocket 连接关闭，3秒后重连...');
                statusDiv.textContent = '🔴 连接已断开';
                distanceDiv.textContent = '--';
                mainBody.style.backgroundColor = '#f0f0f0';
                setTimeout(connectWebSocket, 3000);
            };
        }

        // 你之前的 updateUI 函数...
        function updateUI(data) {
             distanceDiv.textContent = data.distance ? data.distance.toFixed(1) : '--';
            const status = data.status;
            
            if (status === 'Obstacle') {
                statusDiv.textContent = '⚠️ 前方障碍物! ⚠️';
                statusDiv.style.color = 'white';
                mainBody.style.backgroundColor = '#ff4d4d'; // 红色背景
            } else if (status === 'Clear') {
                statusDiv.textContent = '✅ 前方安全';
                statusDiv.style.color = 'white';
                mainBody.style.backgroundColor = '#4CAF50'; // 绿色背景
            }
            
            detailInfo.innerHTML = `水平角度: ${data.angle?.toFixed(1)}°<br>水平像素: ${data.x_pixel?.toFixed(1)}`;
        }

        window.onload = connectWebSocket;
    </script>
</body>
</html>
)rawliteral";

#endif