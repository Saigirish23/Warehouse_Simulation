#!/usr/bin/env python3
"""
Competition Web Server for Robot Monitoring and QR Data

COMPETITION REQUIREMENTS:
- Display mission status (current rack, progress %, elapsed time, QR count)
- Filter QR codes by rack
- Organize data by rack → shelf → items
- Live camera feed endpoint
- Real-time system monitoring

Author: Competition Team
"""

from flask import Flask, jsonify, send_file, render_template_string, Response
import json
import os
from datetime import datetime
from typing import Dict, List
import cv2

app = Flask(__name__)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
QR_JSON_FILE = os.path.join(BASE_DIR, '../../../scanned_qrs.json')
CHECKPOINT_FILE = os.path.join(BASE_DIR, '../../../checkpoint.json')

# Global camera for live feed
camera = None


def get_camera():
    """Get or initialize camera for live feed."""
    global camera
    if camera is None:
        camera = cv2.VideoCapture(0)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    return camera


def load_qr_data() -> Dict:
    """Load QR code database."""
    try:
        with open(QR_JSON_FILE, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        return {"qr_codes": []}
    except Exception as e:
        print(f"Error loading QR data: {e}")
        return {"qr_codes": []}


def load_checkpoint() -> Dict:
    """Load mission checkpoint data."""
    try:
        with open(CHECKPOINT_FILE, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        return {}
    except Exception as e:
        print(f"Error loading checkpoint: {e}")
        return {}


@app.route("/")
def home():
    """Serve main dashboard HTML."""
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Warehouse Robot Competition Dashboard</title>
        <meta http-equiv="refresh" content="5">
        <style>
            body { font-family: Arial; margin: 20px; background: #f0f0f0; }
            .header { background: #2c3e50; color: white; padding: 20px; border-radius: 5px; }
            .stats { display: flex; gap: 20px; margin: 20px 0; }
            .stat-card { background: white; padding: 20px; border-radius: 5px; flex: 1; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
            .stat-value { font-size: 36px; font-weight: bold; color: #3498db; }
            .stat-label { color: #7f8c8d; }
            .data-table { background: white; padding: 20px; border-radius: 5px; margin: 20px 0; }
            table { width: 100%; border-collapse: collapse; }
            th { background: #34495e; color: white; padding: 10px; text-align: left; }
            td { padding: 10px; border-bottom: 1px solid #ecf0f1; }
            tr:hover { background: #f8f9fa; }
            .time-warning { color: #e74c3c; font-weight: bold; }
            .time-ok { color: #27ae60; }
            .progress-bar { background: #ecf0f1; height: 30px; border-radius: 5px; overflow: hidden; }
            .progress-fill { background: #3498db; height: 100%; transition: width 0.3s; }
        </style>
    </head>
    <body>
        <div class="header">
            <h1>🤖 Warehouse Robot Competition Dashboard</h1>
            <p>Real-time Mission Monitoring System</p>
        </div>
        
        <div class="stats">
            <div class="stat-card">
                <div class="stat-label">Mission Progress</div>
                <div class="stat-value" id="progress">--%</div>
            </div>
            <div class="stat-card">
                <div class="stat-label">Time Elapsed</div>
                <div class="stat-value" id="elapsed">--:--</div>
            </div>
            <div class="stat-card">
                <div class="stat-label">Time Remaining</div>
                <div class="stat-value" id="remaining">--:--</div>
            </div>
            <div class="stat-card">
                <div class="stat-label">QR Codes Scanned</div>
                <div class="stat-value" id="qr-count">0</div>
            </div>
        </div>
        
        <div class="data-table">
            <h2>📦 Quick Links</h2>
            <p><a href="/status">Mission Status (JSON)</a></p>
            <p><a href="/qrs">All QR Codes (JSON)</a></p>
            <p><a href="/qrs/R01">Rack R01 QR Codes</a></p>
            <p><a href="/qrs/organized">Organized by Rack/Shelf</a></p>
            <p><a href="/video_feed">Live Camera Feed</a></p>
        </div>
        
        <script>
            // Auto-refresh data
            setInterval(() => {
                fetch('/status')
                    .then(r => r.json())
                    .then(data => {
                        document.getElementById('progress').textContent = data.progress_percent + '%';
                        document.getElementById('elapsed').textContent = formatTime(data.elapsed_seconds);
                        document.getElementById('remaining').textContent = formatTime(data.remaining_seconds);
                        document.getElementById('qr-count').textContent = data.total_qr_scanned;
                    });
            }, 2000);
            
            function formatTime(seconds) {
                const mins = Math.floor(seconds / 60);
                const secs = Math.floor(seconds % 60);
                return mins + ':' + (secs < 10 ? '0' : '') + secs;
            }
        </script>
    </body>
    </html>
    """
    return html


@app.route("/status")
def get_status():
    """
    Get current mission status.
    
    COMPETITION REQUIREMENT:
    - Current rack
    - Progress percentage
    - Elapsed time
    - QR count
    """
    checkpoint = load_checkpoint()
    qr_data = load_qr_data()
    
    current_waypoint = checkpoint.get('current_waypoint_index', 0)
    cumulative_time = checkpoint.get('cumulative_time_sec', 0.0)
    racks_completed = checkpoint.get('racks_completed', [])
    total_qr = checkpoint.get('total_qr_scanned', 0)
    
    # Calculate current rack
    if current_waypoint < len(racks_completed):
        current_rack = racks_completed[current_waypoint].get('rack_id', 'Unknown')
    else:
        current_rack = f'R{current_waypoint+1:02d}'
    
    # Calculate progress (assume 5 racks for competition)
    total_racks = 5
    progress_pct = (len(racks_completed) / total_racks) * 100
    
    # Time calculations
    max_time = 1200  # 20 minutes
    remaining_time = max(0, max_time - cumulative_time)
    
    status = {
        'current_rack': current_rack,
        'progress_percent': round(progress_pct, 1),
        'elapsed_seconds': round(cumulative_time, 1),
        'remaining_seconds': round(remaining_time, 1),
        'racks_completed': len(racks_completed),
        'total_racks': total_racks,
        'total_qr_scanned': total_qr,
        'time_warning': remaining_time < 300,  # <5 min
        'mission_complete': len(racks_completed) >= total_racks
    }
    
    return jsonify(status)


@app.route("/qrs")
def get_all_qrs():
    """Get all scanned QR codes."""
    data = load_qr_data()
    return jsonify(data)


@app.route("/qrs/<rack_id>")
def get_qrs_by_rack(rack_id):
    """
    Get QR codes filtered by rack ID.
    
    COMPETITION REQUIREMENT: Filter by rack
    
    Args:
        rack_id: Rack identifier (e.g., R01, R02)
    """
    data = load_qr_data()
    
    # Filter by rack
    filtered = [
        qr for qr in data.get('qr_codes', [])
        if qr.get('rack_id') == rack_id or qr.get('expected_rack') == rack_id
    ]
    
    return jsonify({
        'rack_id': rack_id,
        'qr_count': len(filtered),
        'qr_codes': filtered
    })


@app.route("/qrs/organized")
def get_qrs_organized():
    """
    Get QR codes organized by rack → shelf → items.
    
    COMPETITION REQUIREMENT: Hierarchical organization
    """
    data = load_qr_data()
    
    # Organize data structure
    organized = {}
    
    for qr in data.get('qr_codes', []):
        rack_id = qr.get('rack_id', 'Unknown')
        shelf_id = qr.get('shelf_id', 'Unknown')
        
        # Initialize rack if needed
        if rack_id not in organized:
            organized[rack_id] = {
                'rack_id': rack_id,
                'shelves': {},
                'total_items': 0
            }
        
        # Initialize shelf if needed
        if shelf_id not in organized[rack_id]['shelves']:
            organized[rack_id]['shelves'][shelf_id] = {
                'shelf_id': shelf_id,
                'items': []
            }
        
        # Add item
        organized[rack_id]['shelves'][shelf_id]['items'].append({
            'item_code': qr.get('item_code'),
            'full_code': qr.get('full_code'),
            'height_cm': qr.get('height_cm'),
            'timestamp': qr.get('timestamp')
        })
        
        organized[rack_id]['total_items'] += 1
    
    return jsonify(organized)


@app.route("/video_feed")
def video_feed():
    """
    Live camera feed endpoint.
    
    COMPETITION REQUIREMENT: Live camera view
    """
    def generate():
        """Generate video stream."""
        cam = get_camera()
        
        while True:
            success, frame = cam.read()
            if not success:
                break
            
            # Encode frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            
            # Yield frame in multipart format
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    
    return Response(generate(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route("/health")
def health_check():
    """Health check endpoint."""
    return jsonify({
        'status': 'healthy',
        'timestamp': datetime.now().isoformat()
    })


if __name__ == '__main__':
    print("=" * 60)
    print("Competition Web Server Starting")
    print("=" * 60)
    print("Dashboard: http://0.0.0.0:5000/")
    print("Status API: http://0.0.0.0:5000/status")
    print("QR Data: http://0.0.0.0:5000/qrs")
    print("Video Feed: http://0.0.0.0:5000/video_feed")
    print("=" * 60)
    
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
