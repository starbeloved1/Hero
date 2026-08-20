from flask import Flask, request, render_template, jsonify, Response
import json
import os
import logging
import requests
import subprocess
from datetime import datetime
from flask_cors import CORS
import time
import hashlib

# 日志配置
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler("server.log"),
        logging.StreamHandler()
    ]
)

app = Flask(__name__)
CORS(app)

# 动态配置
CONFIG_PATH = os.getenv('CONFIG_PATH', '/home/nvidia/2025HeroAimYLMT/src/utils/tools/init.json')
VIDEO_SOURCE = os.getenv('VIDEO_SOURCE', 'http://192.168.1.50:8081/stream')

def load_config():
    """加载配置文件（自动创建缺失文件）"""
    try:
        if not os.path.exists(CONFIG_PATH):
            os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
            with open(CONFIG_PATH, 'w') as f:
                json.dump({"initialized": False}, f)
            logging.info(f"创建默认配置文件: {CONFIG_PATH}")
        
        with open(CONFIG_PATH, 'r') as f:
            return json.load(f)
    except Exception as e:
        logging.error(f"配置操作失败: {str(e)}")
        return {"error": str(e)}

def save_config(data):
    """保存配置文件"""
    try:
        with open(CONFIG_PATH, 'w') as f:
            json.dump(data, f, indent=4, ensure_ascii=False)
        return True
    except Exception as e:
        logging.error(f"保存失败: {str(e)}")
        return False

@app.route('/')
def index():
    return render_template('editor.html', 
                         server_time=datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

@app.route('/api/config', methods=['GET'])
def get_config():
    config_data = load_config()
    if "error" in config_data:
        return jsonify(config_data), 500
    return jsonify(config_data)

@app.route('/api/config', methods=['POST'])
def update_config():
    try:
        new_config = request.get_json()
        if not new_config:
            return jsonify({"error": "空请求体"}), 400
        
        if save_config(new_config):
            return jsonify({"status": "success"})
        return jsonify({"error": "保存失败"}), 500
    except Exception as e:
        logging.error(f"服务器错误: {str(e)}")
        return jsonify({"error": str(e)}), 500

@app.route('/video_feed')
def video_feed():
    def generate():
        try:
            resp = requests.get(VIDEO_SOURCE, stream=True, timeout=10)
            if resp.status_code == 200:
                for chunk in resp.iter_content(chunk_size=1024 * 512):
                    yield chunk
            else:
                logging.warning(f"视频源异常: {resp.status_code}")
        except Exception as e:
            logging.error(f"视频流错误: {str(e)}")
    
    return Response(
        generate(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={'Cache-Control': 'no-cache'}
    )
if __name__ == '__main__':
    app.run(
        host=os.getenv('FLASK_HOST', '0.0.0.0'),
        port=int(os.getenv('FLASK_PORT', 8080)),
        debug=False,
        threaded=True,
        use_reloader=False
    )