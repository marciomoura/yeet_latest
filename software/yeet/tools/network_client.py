#!/usr/bin/env python3
"""
TCP Client for MCU Network Communication
Connects to embedded device, sends commands, receives streaming data,
and serves real-time dashboard via HTTP.

Protocol:
- Commands (text): START, STOP, AMP:X.X, NOISE:X.X
- Data packets (binary): Variable length (up to 1460 bytes)
  [0xDEADBEEF][packet_count][start_sample_index][num_samples][N x float values][checksum]
  Typically contains 361 samples per packet for maximum throughput
"""

import socket
import struct
import threading
import time
import json
from http.server import HTTPServer, BaseHTTPRequestHandler
from collections import deque
from urllib.parse import urlparse, parse_qs
import argparse


class MCUClient:
    """TCP client for communicating with embedded MCU"""
    
    def __init__(self, host='192.168.2.100', port=7000, buffer_size=10000):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        self.running = False
        
        # Data buffer for streaming samples (thread-safe with lock)
        # Using a larger buffer for bulk data recording
        self.data_buffer = deque(maxlen=buffer_size)
        self.buffer_lock = threading.Lock()
        
        # Statistics
        self.packets_received = 0
        self.packets_lost = 0
        self.last_packet_count = -1
        self.total_samples_received = 0
        
    def connect(self):
        """Establish TCP connection to MCU"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)  # 5 second timeout
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f"✓ Connected to MCU at {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close TCP connection"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.connected = False
        print("✓ Disconnected from MCU")
    
    def send_command(self, command):
        """Send text command to MCU"""
        if not self.connected:
            print("✗ Not connected. Use connect() first.")
            return False
        
        try:
            # Add newline terminator if not present
            if not command.endswith('\n'):
                command += '\n'
            
            self.socket.sendall(command.encode('ascii'))
            print(f"→ Sent command: {command.strip()}")
            return True
        except Exception as e:
            print(f"✗ Send failed: {e}")
            self.connected = False
            return False
    
    def start_streaming(self):
        """Send START command to begin data streaming"""
        return self.send_command("START")
    
    def stop_streaming(self):
        """Send STOP command to halt data streaming"""
        return self.send_command("STOP")
    
    def set_amplitude(self, amplitude):
        """Set signal amplitude parameter"""
        return self.send_command(f"AMP:{amplitude:.2f}")
    
    def set_noise(self, noise_level):
        """Set noise level parameter"""
        return self.send_command(f"NOISE:{noise_level:.3f}")
    
    def receive_loop(self):
        """Continuous receive loop for streaming data packets (runs in thread)"""
        self.running = True
        packet_buffer = b''
        HEADER_SIZE = 14  # 4 (header) + 4 (count) + 4 (start_idx) + 2 (num_samples)
        CHECKSUM_SIZE = 2
        
        while self.running and self.connected:
            try:
                # Read data from socket (non-blocking)
                self.socket.settimeout(0.1)  # 100ms timeout for responsiveness
                chunk = self.socket.recv(8192)  # Larger recv buffer for bulk data
                
                if not chunk:
                    # Connection closed by remote
                    print("✗ Connection closed by MCU")
                    self.connected = False
                    break
                
                packet_buffer += chunk
                
                # Process complete packets (variable length)
                while len(packet_buffer) >= HEADER_SIZE + CHECKSUM_SIZE:
                    # Look for packet header 0xDEADBEEF
                    header_idx = packet_buffer.find(b'\xDE\xAD\xBE\xEF')
                    
                    if header_idx == -1:
                        # No valid header found, keep last 4 bytes in case header is split
                        packet_buffer = packet_buffer[-4:] if len(packet_buffer) >= 4 else b''
                        break
                    
                    # Discard any data before header
                    if header_idx > 0:
                        packet_buffer = packet_buffer[header_idx:]
                    
                    # Check if we have enough data to read header
                    if len(packet_buffer) < HEADER_SIZE:
                        break
                    
                    # Parse header to get packet length
                    num_samples = struct.unpack('>H', packet_buffer[12:14])[0]
                    
                    # Sanity check: num_samples should be reasonable (expect 361 for bulk mode)
                    if num_samples == 0 or num_samples > 1000:
                        print(f"✗ Invalid num_samples={num_samples} at offset {header_idx}, likely false header - skipping 1 byte")
                        packet_buffer = packet_buffer[1:]  # Skip 1 byte and re-scan
                        continue
                    
                    expected_packet_size = HEADER_SIZE + (num_samples * 4) + CHECKSUM_SIZE
                    
                    # Check if we have complete packet
                    if len(packet_buffer) < expected_packet_size:
                        break
                    
                    # Extract packet
                    packet = packet_buffer[:expected_packet_size]
                    packet_buffer = packet_buffer[expected_packet_size:]
                    
                    # Parse packet
                    self._parse_packet(packet)
                    
            except socket.timeout:
                # Timeout is normal, continue loop
                continue
            except Exception as e:
                print(f"✗ Receive error: {e}")
                self.connected = False
                break
        
        self.running = False
        print("✓ Receive loop stopped")
    
    def _parse_packet(self, packet):
        """Parse binary data packet from MCU (variable length bulk format)"""
        try:
            # Verify header
            if packet[0:4] != b'\xDE\xAD\xBE\xEF':
                return
            
            # Unpack packet header fields (big-endian)
            packet_count = struct.unpack('>I', packet[4:8])[0]
            start_sample_idx = struct.unpack('>I', packet[8:12])[0]
            num_samples = struct.unpack('>H', packet[12:14])[0]
            
            # Unpack all float values
            signal_values = []
            offset = 14
            for i in range(num_samples):
                signal_value = struct.unpack('>f', packet[offset:offset+4])[0]
                signal_values.append(signal_value)
                offset += 4
            
            # Get checksum
            checksum_rx = struct.unpack('>H', packet[offset:offset+2])[0]
            
            # Verify checksum (sum of all bytes before checksum)
            checksum_calc = sum(packet[:offset]) & 0xFFFF
            if checksum_calc != checksum_rx:
                print(f"✗ Checksum mismatch: expected {checksum_calc:04X}, got {checksum_rx:04X}")
                print(f"  → Packet #{packet_count}, num_samples={num_samples}, packet_len={len(packet)}, expected_len={offset+2}")
                print(f"  → Header bytes: {packet[0:4].hex()}, First float: {packet[14:18].hex()}")
                # If checksum is 0x3F80 or 0xBEEF, we're likely desynchronized (reading float data or header as checksum)
                if checksum_rx == 0x3F80:
                    print(f"  ⚠ WARNING: Checksum 0x3F80 = float(1.0) - likely reading float data as checksum (DESYNC!)")
                elif checksum_rx & 0xFF00 == 0xBE00 or checksum_rx & 0x00FF == 0xEF:
                    print(f"  ⚠ WARNING: Checksum contains 0xBE/0xEF - likely reading header as checksum (DESYNC!)")
                return
            
            # Check for packet loss
            if self.last_packet_count >= 0:
                expected = self.last_packet_count + 1
                if packet_count != expected:
                    lost = packet_count - expected
                    self.packets_lost += lost
                    print(f"⚠ Packet loss detected: {lost} packets")
            
            self.last_packet_count = packet_count
            self.packets_received += 1
            self.total_samples_received += num_samples
            
            # Store individual data points in buffer (thread-safe)
            # Each sample gets its own entry for easy plotting
            with self.buffer_lock:
                for i, value in enumerate(signal_values):
                    data_point = {
                        'packet_count': packet_count,
                        'sample_index': start_sample_idx + i,
                        'value': value
                    }
                    self.data_buffer.append(data_point)
            
            # Print periodic status
            if self.packets_received % 10 == 0:
                print(f"✓ Received {self.packets_received} packets, "
                      f"{self.total_samples_received} samples, "
                      f"Lost: {self.packets_lost}, "
                      f"Latest: {signal_values[-1]:.3f}")
                
        except Exception as e:
            print(f"✗ Packet parse error: {e}")
            import traceback
            traceback.print_exc()
    
    def get_recent_data(self, max_points=100):
        """Get recent data points for plotting (thread-safe)"""
        with self.buffer_lock:
            # Return last N points
            n = min(max_points, len(self.data_buffer))
            return list(self.data_buffer)[-n:] if n > 0 else []
    
    def get_data_since(self, last_sample_index, max_points=1000):
        """Get data points newer than the provided sample index."""
        with self.buffer_lock:
            if len(self.data_buffer) == 0:
                return []

            # Collect points with sample_index greater than last_sample_index
            result = [point for point in self.data_buffer if point['sample_index'] > last_sample_index]

            if max_points is not None and len(result) > max_points:
                result = result[-max_points:]

            return result

    def get_stats(self):
        """Get connection statistics"""
        return {
            'connected': self.connected,
            'packets_received': self.packets_received,
            'packets_lost': self.packets_lost,
            'buffer_size': len(self.data_buffer),
            'total_samples': self.total_samples_received
        }


class DashboardHandler(BaseHTTPRequestHandler):
    """HTTP request handler for web dashboard"""
    
    mcu_client = None  # Set by main thread
    
    def do_GET(self):
        """Handle GET requests"""
        parsed_url = urlparse(self.path)
        path = parsed_url.path
        query = parse_qs(parsed_url.query)

        if path == '/':
            # Serve main dashboard HTML
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(get_dashboard_html().encode())
            
        elif path == '/api/data':
            # Determine if caller requested data newer than a specific sample index
            since_param = query.get('since')
            limit_param = query.get('limit')

            if since_param is not None:
                try:
                    last_sample_index = int(since_param[0])
                except ValueError:
                    last_sample_index = -1
                try:
                    limit_value = int(limit_param[0]) if limit_param else 1000
                except ValueError:
                    limit_value = 1000
                data = self.mcu_client.get_data_since(last_sample_index, max_points=limit_value)
            else:
                try:
                    limit_value = int(limit_param[0]) if limit_param else 500
                except ValueError:
                    limit_value = 500
                data = self.mcu_client.get_recent_data(max_points=limit_value)

            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(data).encode())
            
        elif path == '/api/stats':
            # Return statistics as JSON
            stats = self.mcu_client.get_stats()
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(stats).encode())
            
        else:
            self.send_error(404)
    
    def do_POST(self):
        """Handle POST requests (commands)"""
        parsed_url = urlparse(self.path)
        if parsed_url.path == '/api/command':
            # Read command from request body
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            command_data = json.loads(post_data.decode())
            
            command = command_data.get('command', '')
            success = False
            
            if command == 'START':
                success = self.mcu_client.start_streaming()
            elif command == 'STOP':
                success = self.mcu_client.stop_streaming()
            elif command.startswith('AMP:'):
                try:
                    amp = float(command.split(':')[1])
                    success = self.mcu_client.set_amplitude(amp)
                except:
                    pass
            elif command.startswith('NOISE:'):
                try:
                    noise = float(command.split(':')[1])
                    success = self.mcu_client.set_noise(noise)
                except:
                    pass
            
            # Send response
            self.send_response(200 if success else 500)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            response = {'success': success, 'command': command}
            self.wfile.write(json.dumps(response).encode())
        else:
            self.send_error(404)
    
    def log_message(self, format, *args):
        """Suppress default HTTP logging"""
        pass


def get_dashboard_html():
    """Generate HTML for web dashboard"""
    return """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>MCU Network Dashboard</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            border-radius: 15px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
            overflow: hidden;
        }
        .header {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 25px;
            text-align: center;
        }
        .header h1 {
            font-size: 2em;
            margin-bottom: 10px;
        }
        .status {
            display: inline-block;
            padding: 5px 15px;
            border-radius: 20px;
            background: rgba(255,255,255,0.2);
            font-size: 0.9em;
        }
        .status.connected { background: #10b981; }
        .status.disconnected { background: #ef4444; }
        .controls {
            padding: 25px;
            border-bottom: 2px solid #f0f0f0;
            display: flex;
            gap: 15px;
            flex-wrap: wrap;
            align-items: center;
        }
        button {
            padding: 12px 24px;
            border: none;
            border-radius: 8px;
            font-size: 1em;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s;
        }
        .btn-primary {
            background: #10b981;
            color: white;
        }
        .btn-primary:hover { background: #059669; }
        .btn-danger {
            background: #ef4444;
            color: white;
        }
        .btn-danger:hover { background: #dc2626; }
        .param-group {
            display: flex;
            align-items: center;
            gap: 10px;
            padding: 10px;
            background: #f9fafb;
            border-radius: 8px;
        }
        input[type="number"] {
            padding: 8px 12px;
            border: 2px solid #e5e7eb;
            border-radius: 6px;
            font-size: 1em;
            width: 100px;
        }
        .chart-container {
            padding: 25px;
            position: relative;
            height: 400px;
        }
        .stats {
            padding: 25px;
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
            background: #f9fafb;
        }
        .stat-card {
            background: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.1);
        }
        .stat-label {
            color: #6b7280;
            font-size: 0.9em;
            margin-bottom: 5px;
        }
        .stat-value {
            font-size: 1.8em;
            font-weight: 700;
            color: #1f2937;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🚀 MCU Network Dashboard</h1>
            <span class="status" id="status">Connecting...</span>
        </div>
        
        <div class="controls">
            <button class="btn-primary" onclick="sendCommand('START')">▶ Start Streaming</button>
            <button class="btn-danger" onclick="sendCommand('STOP')">⏸ Stop Streaming</button>
            
            <div class="param-group">
                <label>Amplitude:</label>
                <input type="number" id="amplitude" value="1.0" step="0.1" min="0" max="10">
                <button class="btn-primary" onclick="setAmplitude()">Set</button>
            </div>
            
            <div class="param-group">
                <label>Noise:</label>
                <input type="number" id="noise" value="0.1" step="0.01" min="0" max="1">
                <button class="btn-primary" onclick="setNoise()">Set</button>
            </div>
            
            <button class="btn-primary" onclick="exportToCSV()">📊 Export CSV</button>
        </div>
        
        <div class="chart-container">
            <canvas id="signalChart"></canvas>
        </div>
        
        <div class="stats">
            <div class="stat-card">
                <div class="stat-label">Packets Received</div>
                <div class="stat-value" id="packets-rx">0</div>
            </div>
            <div class="stat-card">
                <div class="stat-label">Packets Lost</div>
                <div class="stat-value" id="packets-lost">0</div>
            </div>
            <div class="stat-card">
                <div class="stat-label">Total Samples</div>
                <div class="stat-value" id="total-samples">0</div>
            </div>
            <div class="stat-card">
                <div class="stat-label">Buffer Size</div>
                <div class="stat-value" id="buffer-size">0</div>
            </div>
            <div class="stat-card">
                <div class="stat-label">Latest Value</div>
                <div class="stat-value" id="latest-value">--</div>
            </div>
        </div>
    </div>
    
    <script>
    // Global data storage for export and continuous recording
    const MAX_BUFFER_POINTS = 10000;   // Client-side retention cap (matches default server buffer)
    const MAX_DISPLAY_POINTS = 5000;   // Limit chart points for performance
    let allDataPoints = [];
    let lastSampleIndex = -1;
        
        // Initialize Chart.js for single-channel real-time recorder
        const ctx = document.getElementById('signalChart').getContext('2d');
        
        const chart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Signal Value',
                    data: [],
                    borderColor: '#667eea',
                    backgroundColor: 'transparent',
                    borderWidth: 1.5,
                    tension: 0.1,
                    pointRadius: 0
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                interaction: {
                    mode: 'nearest',
                    intersect: false
                },
                plugins: {
                    legend: { display: true }
                },
                scales: {
                    x: {
                        title: { display: true, text: 'Sample Index' },
                        ticks: { maxTicksLimit: 15 }
                    },
                    y: {
                        title: { display: true, text: 'Signal Value' }
                    }
                }
            }
        });
        
        // Send command to server
        async function sendCommand(cmd) {
            try {
                const response = await fetch('/api/command', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ command: cmd })
                });
                const result = await response.json();
                console.log('Command result:', result);
            } catch (err) {
                console.error('Command error:', err);
            }
        }
        
        function setAmplitude() {
            const amp = document.getElementById('amplitude').value;
            sendCommand(`AMP:${amp}`);
        }
        
        function setNoise() {
            const noise = document.getElementById('noise').value;
            sendCommand(`NOISE:${noise}`);
        }
        
        // Export data to CSV
        function exportToCSV() {
            if (allDataPoints.length === 0) {
                alert('No data to export! Start streaming first.');
                return;
            }
            
            // Build CSV content
            let csv = 'Packet Count,Sample Index,Signal Value\\n';
            
            // Add data rows
            allDataPoints.forEach(point => {
                csv += `${point.packet_count},${point.sample_index},${point.value.toFixed(6)}\\n`;
            });
            
            // Create download link
            const blob = new Blob([csv], { type: 'text/csv' });
            const url = window.URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.setAttribute('hidden', '');
            a.setAttribute('href', url);
            a.setAttribute('download', `mcu_recording_${Date.now()}.csv`);
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            
            console.log(`✓ Exported ${allDataPoints.length} samples to CSV`);
        }
        
        // Update chart with latest data (continuous recorder mode)
        async function updateChart() {
            try {
                let endpoint = '/api/data';
                if (lastSampleIndex >= 0) {
                    endpoint += `?since=${lastSampleIndex}&limit=${MAX_BUFFER_POINTS}`;
                } else {
                    endpoint += `?limit=${MAX_BUFFER_POINTS}`;
                }

                const response = await fetch(endpoint);
                const newData = await response.json();

                if (newData.length > 0) {
                    // Ensure records are ordered by sample index
                    newData.sort((a, b) => a.sample_index - b.sample_index);

                    // Append new data to the local buffer
                    allDataPoints = allDataPoints.concat(newData);

                    // Enforce maximum client-side buffer size
                    if (allDataPoints.length > MAX_BUFFER_POINTS) {
                        allDataPoints = allDataPoints.slice(-MAX_BUFFER_POINTS);
                    }

                    // Update last processed sample index
                    lastSampleIndex = allDataPoints[allDataPoints.length - 1].sample_index;
                
                    // Determine subset for chart display
                    const displayData = allDataPoints.slice(-MAX_DISPLAY_POINTS);

                    chart.data.labels = displayData.map(d => d.sample_index);
                    chart.data.datasets[0].data = displayData.map(d => d.value);

                    chart.update('none');

                    // Update latest value display
                    const latest = displayData[displayData.length - 1];
                    document.getElementById('latest-value').textContent = 
                        latest.value.toFixed(3);
                }
            } catch (err) {
                console.error('Data fetch error:', err);
            }
        }
        
        // Update statistics
        async function updateStats() {
            try {
                const response = await fetch('/api/stats');
                const stats = await response.json();
                
                document.getElementById('packets-rx').textContent = 
                    stats.packets_received;
                document.getElementById('packets-lost').textContent = 
                    stats.packets_lost;
                document.getElementById('buffer-size').textContent = 
                    stats.buffer_size;
                
                // Show total samples received
                if (stats.total_samples !== undefined) {
                    document.getElementById('total-samples').textContent = 
                        stats.total_samples.toLocaleString();
                }
                
                const statusEl = document.getElementById('status');
                if (stats.connected) {
                    statusEl.textContent = '● Connected';
                    statusEl.className = 'status connected';
                } else {
                    statusEl.textContent = '● Disconnected';
                    statusEl.className = 'status disconnected';
                }
            } catch (err) {
                console.error('Stats fetch error:', err);
            }
        }
        
        // Refresh data periodically (continuous polling)
        setInterval(updateChart, 100);   // 10 Hz chart update
        setInterval(updateStats, 500);   // 2 Hz stats update
        
        // Initial update
        updateChart();
        updateStats();
    </script>
</body>
</html>"""


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='MCU Network Client & Dashboard')
    parser.add_argument('--host', default='192.168.2.100',
                      help='MCU IP address (default: 192.168.2.100)')
    parser.add_argument('--port', type=int, default=7000,
                      help='MCU TCP port (default: 7000)')
    parser.add_argument('--http-port', type=int, default=8080,
                      help='HTTP dashboard port (default: 8080)')
    parser.add_argument('--buffer-size', type=int, default=10000,
                      help='Data buffer size (default: 10000 samples)')
    args = parser.parse_args()
    
    print("=" * 60)
    print("MCU Network Client & Dashboard")
    print("=" * 60)
    print(f"MCU Address: {args.host}:{args.port}")
    print(f"Dashboard: http://localhost:{args.http_port}")
    print("=" * 60)
    
    # Create MCU client
    client = MCUClient(host=args.host, port=args.port, buffer_size=args.buffer_size)
    
    # Connect to MCU
    if not client.connect():
        print("\n✗ Failed to connect to MCU. Please check:")
        print("  1. MCU is powered on and running")
        print("  2. Network cable is connected")
        print("  3. IP address is correct")
        print(f"  4. Firewall allows connection to {args.host}:{args.port}")
        return 1
    
    # Start receive thread
    receive_thread = threading.Thread(target=client.receive_loop, daemon=True)
    receive_thread.start()
    
    # Set up HTTP server for dashboard
    DashboardHandler.mcu_client = client
    httpd = HTTPServer(('localhost', args.http_port), DashboardHandler)
    
    print(f"\n✓ Dashboard running at http://localhost:{args.http_port}")
    print("✓ Press Ctrl+C to exit\n")
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n\n✓ Shutting down...")
        client.stop_streaming()
        client.disconnect()
        httpd.shutdown()
        return 0


if __name__ == '__main__':
    exit(main())
