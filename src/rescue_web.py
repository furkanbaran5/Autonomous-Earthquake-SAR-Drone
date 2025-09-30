#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rescue Data Web Viewer
Web tabanlı modern kurtarma verisi görüntüleyicisi
"""

import json
import os
import threading
import time
import webbrowser
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse


class RescueWebViewer(BaseHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        self.data_dir = os.path.expanduser("~/rescue_archive")
        self.database_file = os.path.join(self.data_dir, "rescue_database.json")
        super().__init__(*args, **kwargs)

    def do_GET(self):
        """GET isteklerini işle"""
        if self.path == '/' or self.path == '/index.html':
            self.serve_main_page()
        elif self.path == '/api/data':
            self.serve_data_api()
        elif self.path.startswith('/api/image/'):
            self.serve_image_api()
        elif self.path == '/style.css':
            self.serve_css()
        else:
            self.send_error(404)

    def serve_main_page(self):
        """Ana sayfa HTML"""
        html = """<!DOCTYPE html>
<html lang="tr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🚁 Enkaz Kurtarma Sistemi</title>
    <link rel="stylesheet" href="/style.css">
</head>
<body>
    <div class="container">
        <header class="header">
            <div class="header-content">
                <h1>🚁 ENKAZ KURTARMA SİSTEMİ</h1>
                <p>Modern Web Görüntüleyici</p>
            </div>
            <div class="status" id="status">🔄 Yükleniyor...</div>
        </header>

        <div class="main-content">
            <div class="sidebar">
                <div class="card">
                    <h3>📊 İstatistikler</h3>
                    <div id="stats">Yükleniyor...</div>
                </div>

                <div class="card">
                    <h3>🧭 Navigasyon</h3>
                    <div class="nav-controls">
                        <button id="prevBtn" class="btn btn-primary">⬅️ Önceki</button>
                        <button id="nextBtn" class="btn btn-primary">➡️ Sonraki</button>
                        <button id="refreshBtn" class="btn btn-success">🔄 Yenile</button>
                    </div>
                </div>

                <div class="card">
                    <h3>🏢 Bina Filtresi</h3>
                    <select id="buildingFilter" class="select">
                        <option value="all">Tümü</option>
                    </select>
                </div>
            </div>

            <div class="content">
                <div class="image-card">
                    <div class="card-header">
                        <h3>📷 Görüntü Detayı</h3>
                        <span id="navInfo">0 / 0</span>
                    </div>
                    <div class="image-container" id="imageContainer">
                        <div class="no-data">📋 Veri Yükleniyor...</div>
                    </div>
                </div>

                <div class="details-card">
                    <h3>📋 Tespit Detayları</h3>
                    <div id="details" class="details-content">
                        Yükleniyor...
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script>
        let currentIndex = 0;
        let rescueData = [];
        let filteredData = [];

        // Sayfa yüklendiğinde başlat
        document.addEventListener('DOMContentLoaded', function() {
            loadData();
            setupEventListeners();

            // Otomatik yenileme - 5 saniyede bir
            setInterval(loadData, 5000);
        });

        function setupEventListeners() {
            document.getElementById('prevBtn').addEventListener('click', () => {
                if (currentIndex > 0) {
                    currentIndex--;
                    updateDisplay();
                }
            });

            document.getElementById('nextBtn').addEventListener('click', () => {
                if (currentIndex < filteredData.length - 1) {
                    currentIndex++;
                    updateDisplay();
                }
            });

            document.getElementById('refreshBtn').addEventListener('click', () => {
                loadData();
                showMessage('Veriler yenilendi!');
            });

            document.getElementById('buildingFilter').addEventListener('change', (e) => {
                filterData(e.target.value);
                currentIndex = 0;
                updateDisplay();
            });
        }

        async function loadData() {
            try {
                const response = await fetch('/api/data');
                const data = await response.json();
                rescueData = data.records;

                updateBuildingFilter();
                filterData(document.getElementById('buildingFilter').value);
                updateDisplay();
                updateStatus();
            } catch (error) {
                console.error('Veri yükleme hatası:', error);
                document.getElementById('status').innerHTML = '🔴 Bağlantı Hatası';
            }
        }

        function updateBuildingFilter() {
            const select = document.getElementById('buildingFilter');
            const buildings = new Set();

            rescueData.forEach(record => {
                buildings.add(record.building_name || 'bilinmeyen_alan');
            });

            const currentValue = select.value;
            select.innerHTML = '<option value="all">Tümü</option>';

            Array.from(buildings).sort().forEach(building => {
                const option = document.createElement('option');
                option.value = building;
                option.textContent = building.replace('_', ' ').replace(/\\b\\w/g, l => l.toUpperCase());
                select.appendChild(option);
            });

            select.value = currentValue;
        }

        function filterData(filterValue) {
            if (filterValue === 'all') {
                filteredData = rescueData;
            } else {
                filteredData = rescueData.filter(record =>
                    record.building_name === filterValue
                );
            }
        }

        function updateDisplay() {
            if (filteredData.length === 0) {
                showNoData();
                return;
            }

            if (currentIndex >= filteredData.length) currentIndex = 0;
            if (currentIndex < 0) currentIndex = filteredData.length - 1;

            const record = filteredData[currentIndex];

            showImage(record);
            showDetails(record);
            updateNavigation();
            updateStatistics();
        }

        function showImage(record) {
            const container = document.getElementById('imageContainer');

            if (record.image_saved && record.image_path) {
                // Görüntüyü yükle
                const img = new Image();
                img.className = 'rescue-image';
                img.alt = 'Rescue Image';

                img.onload = function() {
                    container.innerHTML = '';
                    container.appendChild(this);
                };

                img.onerror = function() {
                    console.error('Görüntü yüklenemiyor:', record.id);
                    container.innerHTML = '<div class="error">📷 Görüntü Yüklenemedi</div>';
                };

                // Önce yükleniyor göster
                container.innerHTML = '<div class="no-data">📷 Görüntü Yükleniyor...</div>';

                // Görüntüyü yükle
                img.src = `/api/image/${encodeURIComponent(record.id)}?t=${Date.now()}`;
            } else {
                container.innerHTML = '<div class="no-image">📷 Görüntü Bulunamadı</div>';
            }
        }

        function showDetails(record) {
            const pos = record.position || {};
            const details = `
                <div class="detail-row">
                    <span class="label">🆔 ID:</span>
                    <span class="value">${record.id || 'N/A'}</span>
                </div>
                <div class="detail-row">
                    <span class="label">📍 Koordinatlar:</span>
                    <span class="value">X: ${pos.x?.toFixed(2) || 0}, Y: ${pos.y?.toFixed(2) || 0}, Z: ${pos.z?.toFixed(2) || 0}</span>
                </div>
                <div class="detail-row">
                    <span class="label">🏢 Bina:</span>
                    <span class="value">${record.building || 'Bilinmiyor'}</span>
                </div>
                <div class="detail-row">
                    <span class="label">🏗️ Kat:</span>
                    <span class="value">${record.floor || 'N/A'}. Kat</span>
                </div>
                <div class="detail-row">
                    <span class="label">🕐 Zaman:</span>
                    <span class="value">${formatTimestamp(record.timestamp)}</span>
                </div>
                <div class="detail-row">
                    <span class="label">🎯 Güven:</span>
                    <span class="value">${((record.confidence || 0.5) * 100).toFixed(1)}%</span>
                </div>
            `;
            document.getElementById('details').innerHTML = details;
        }

        function showNoData() {
            document.getElementById('imageContainer').innerHTML = `
                <div class="no-data">
                    📋 Henüz Veri Yok<br><br>
                    Furkan'ın YOLO sistemi çalıştığında<br>
                    veriler burada görünecek
                </div>
            `;
            document.getElementById('details').innerHTML = 'Henüz kurtarma verisi bulunmuyor.';
            document.getElementById('navInfo').textContent = '0 / 0';
        }

        function updateNavigation() {
            document.getElementById('navInfo').textContent = `${currentIndex + 1} / ${filteredData.length}`;

            const prevBtn = document.getElementById('prevBtn');
            const nextBtn = document.getElementById('nextBtn');

            prevBtn.disabled = currentIndex === 0 || filteredData.length <= 1;
            nextBtn.disabled = currentIndex === filteredData.length - 1 || filteredData.length <= 1;
        }

        function updateStatistics() {
            const buildingStats = {};
            rescueData.forEach(record => {
                const building = record.building_name || 'bilinmeyen_alan';
                buildingStats[building] = (buildingStats[building] || 0) + 1;
            });

            let statsHtml = `
                <div class="stat-item">
                    <span class="stat-label">🎯 Toplam:</span>
                    <span class="stat-value">${rescueData.length}</span>
                </div>
            `;

            if (document.getElementById('buildingFilter').value !== 'all') {
                statsHtml += `
                    <div class="stat-item">
                        <span class="stat-label">🔍 Filtrelenmiş:</span>
                        <span class="stat-value">${filteredData.length}</span>
                    </div>
                `;
            }

            Object.entries(buildingStats).sort().forEach(([building, count]) => {
                const buildingName = building.replace('_', ' ').replace(/\\b\\w/g, l => l.toUpperCase());
                statsHtml += `
                    <div class="stat-item">
                        <span class="stat-label">🏢 ${buildingName}:</span>
                        <span class="stat-value">${count}</span>
                    </div>
                `;
            });

            document.getElementById('stats').innerHTML = statsHtml;
        }

        function updateStatus() {
            const status = document.getElementById('status');
            if (rescueData.length > 0) {
                status.innerHTML = `🟢 ${rescueData.length} Tespit Aktif`;
                status.className = 'status active';
            } else {
                status.innerHTML = '🟡 Veri Bekleniyor';
                status.className = 'status waiting';
            }
        }

        function formatTimestamp(timestamp) {
            if (!timestamp) return 'Bilinmiyor';
            try {
                const date = new Date(timestamp);
                return date.toLocaleString('tr-TR');
            } catch {
                return timestamp;
            }
        }

        function showMessage(message) {
            const msgDiv = document.createElement('div');
            msgDiv.className = 'message';
            msgDiv.textContent = message;
            document.body.appendChild(msgDiv);
            setTimeout(() => msgDiv.remove(), 3000);
        }
    </script>
</body>
</html>"""

        self.send_response(200)
        self.send_header('Content-type', 'text/html; charset=utf-8')
        self.end_headers()
        self.wfile.write(html.encode('utf-8'))

    def serve_css(self):
        """CSS stilleri"""
        css = """
* { margin: 0; padding: 0; box-sizing: border-box; }

body {
    font-family: 'Segoe UI', Arial, sans-serif;
    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
    min-height: 100vh;
    color: #333;
}

.container { max-width: 1400px; margin: 0 auto; padding: 20px; }

.header {
    background: rgba(255, 255, 255, 0.95);
    border-radius: 15px;
    padding: 30px;
    margin-bottom: 30px;
    display: flex;
    justify-content: space-between;
    align-items: center;
    box-shadow: 0 10px 30px rgba(0,0,0,0.1);
    backdrop-filter: blur(10px);
}

.header h1 {
    font-size: 28px;
    color: #2c3e50;
    margin-bottom: 5px;
}

.header p {
    color: #7f8c8d;
    font-size: 16px;
}

.status {
    font-weight: bold;
    font-size: 16px;
    padding: 10px 20px;
    border-radius: 25px;
    background: #f39c12;
    color: white;
}

.status.active { background: #27ae60; }
.status.waiting { background: #e67e22; }

.main-content {
    display: flex;
    gap: 30px;
}

.sidebar {
    width: 350px;
    display: flex;
    flex-direction: column;
    gap: 20px;
}

.content {
    flex: 1;
    display: flex;
    flex-direction: column;
    gap: 20px;
}

.card {
    background: rgba(255, 255, 255, 0.95);
    border-radius: 15px;
    padding: 25px;
    box-shadow: 0 10px 30px rgba(0,0,0,0.1);
    backdrop-filter: blur(10px);
}

.card h3 {
    color: #2c3e50;
    margin-bottom: 20px;
    font-size: 18px;
    display: flex;
    align-items: center;
    gap: 10px;
}

.image-card {
    background: rgba(255, 255, 255, 0.95);
    border-radius: 15px;
    padding: 0;
    box-shadow: 0 10px 30px rgba(0,0,0,0.1);
    backdrop-filter: blur(10px);
    flex: 1;
    display: flex;
    flex-direction: column;
}

.card-header {
    padding: 25px;
    border-bottom: 2px solid #ecf0f1;
    display: flex;
    justify-content: space-between;
    align-items: center;
}

.card-header h3 { margin: 0; }

#navInfo {
    font-weight: bold;
    color: #3498db;
    font-size: 16px;
}

.image-container {
    flex: 1;
    padding: 25px;
    display: flex;
    align-items: center;
    justify-content: center;
    min-height: 400px;
}

.rescue-image {
    max-width: 100%;
    max-height: 100%;
    border-radius: 10px;
    box-shadow: 0 5px 15px rgba(0,0,0,0.2);
}

.no-data, .no-image, .error {
    text-align: center;
    color: #7f8c8d;
    font-size: 18px;
    line-height: 1.8;
}

.details-card {
    background: rgba(255, 255, 255, 0.95);
    border-radius: 15px;
    padding: 25px;
    box-shadow: 0 10px 30px rgba(0,0,0,0.1);
    backdrop-filter: blur(10px);
}

.details-content {
    display: flex;
    flex-direction: column;
    gap: 15px;
}

.detail-row {
    display: flex;
    justify-content: space-between;
    padding: 12px 0;
    border-bottom: 1px solid #ecf0f1;
}

.detail-row:last-child { border-bottom: none; }

.label {
    font-weight: bold;
    color: #34495e;
}

.value {
    color: #2c3e50;
}

.nav-controls {
    display: flex;
    flex-direction: column;
    gap: 10px;
}

.btn {
    padding: 12px 20px;
    border: none;
    border-radius: 8px;
    font-size: 14px;
    font-weight: bold;
    cursor: pointer;
    transition: all 0.3s ease;
}

.btn:disabled {
    opacity: 0.5;
    cursor: not-allowed;
}

.btn-primary {
    background: #3498db;
    color: white;
}

.btn-primary:hover:not(:disabled) {
    background: #2980b9;
    transform: translateY(-2px);
}

.btn-success {
    background: #27ae60;
    color: white;
}

.btn-success:hover:not(:disabled) {
    background: #229954;
    transform: translateY(-2px);
}

.select {
    width: 100%;
    padding: 12px;
    border: 2px solid #bdc3c7;
    border-radius: 8px;
    font-size: 14px;
    background: white;
}

.stat-item {
    display: flex;
    justify-content: space-between;
    padding: 8px 0;
    border-bottom: 1px solid #ecf0f1;
}

.stat-item:last-child { border-bottom: none; }

.stat-label {
    color: #7f8c8d;
    font-size: 14px;
}

.stat-value {
    font-weight: bold;
    color: #2c3e50;
}

.message {
    position: fixed;
    top: 20px;
    right: 20px;
    background: #27ae60;
    color: white;
    padding: 15px 25px;
    border-radius: 8px;
    box-shadow: 0 5px 15px rgba(0,0,0,0.2);
    z-index: 1000;
    animation: slideIn 0.3s ease;
}

@keyframes slideIn {
    from { transform: translateX(100%); }
    to { transform: translateX(0); }
}

@media (max-width: 768px) {
    .main-content { flex-direction: column; }
    .sidebar { width: 100%; }
    .container { padding: 10px; }
    .header { padding: 20px; flex-direction: column; gap: 15px; }
}
"""

        self.send_response(200)
        self.send_header('Content-type', 'text/css; charset=utf-8')
        self.end_headers()
        self.wfile.write(css.encode('utf-8'))

    def serve_data_api(self):
        """Veri API'si"""
        try:
            if os.path.exists(self.database_file):
                with open(self.database_file, 'r', encoding='utf-8') as f:
                    all_data = json.load(f)
                    # Sadece görüntülü kayıtlar
                    filtered_data = [r for r in all_data if r.get('image_saved', False)]
            else:
                filtered_data = []

            response = {
                'success': True,
                'records': filtered_data,
                'total': len(filtered_data)
            }
        except Exception as e:
            response = {
                'success': False,
                'error': str(e),
                'records': [],
                'total': 0
            }

        self.send_response(200)
        self.send_header('Content-type', 'application/json; charset=utf-8')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))

    def serve_image_api(self):
        """Görüntü API'si"""
        # URL'den ID'yi al - query parametrelerini temizle
        path_parts = self.path.split('?')[0]  # Query string'i kaldır
        image_id = urllib.parse.unquote(path_parts.split('/')[-1])

        try:
            # Veritabanından ilgili kaydı bul
            if os.path.exists(self.database_file):
                with open(self.database_file, 'r', encoding='utf-8') as f:
                    all_data = json.load(f)

                # ID'ye göre kayıt bul
                record = None
                for r in all_data:
                    if r.get('id') == image_id:
                        record = r
                        break

                if record and record.get('image_path'):
                    image_path = record['image_path']
                    if os.path.exists(image_path):
                        # Görüntü dosyasını oku
                        with open(image_path, 'rb') as img_file:
                            image_data = img_file.read()

                        # MIME type belirle
                        if image_path.lower().endswith('.png'):
                            content_type = 'image/png'
                        else:
                            content_type = 'image/jpeg'

                        self.send_response(200)
                        self.send_header('Content-type', content_type)
                        self.send_header('Content-Length', str(len(image_data)))
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.send_header('Cache-Control', 'no-cache')
                        self.end_headers()
                        self.wfile.write(image_data)
                        return
                    else:
                        print(f"❌ Görüntü dosyası bulunamadı: {image_path}")
                        self.send_error(404, f'Image file not found: {image_path}')
                        return
                else:
                    print(f"❌ Kayıt bulunamadı veya görüntü yolu yok: {image_id}")
                    self.send_error(404, f'Record not found: {image_id}')
                    return
            else:
                print(f"❌ Database dosyası bulunamadı: {self.database_file}")
                self.send_error(404, 'Database not found')
                return

        except Exception as e:
            print(f"❌ Görüntü API hatası: {e}")
            self.send_error(500, f'Server error: {e}')

    def log_message(self, format, *args):
        """Log mesajlarını sustur"""
        pass


def open_browser(url):
    """2 saniye sonra tarayıcıyı aç"""
    time.sleep(2)
    try:
        webbrowser.open(url)
        print(f"🌍 Tarayıcı otomatik olarak açıldı: {url}")
    except Exception as e:
        print(f"⚠️ Tarayıcı açılamadı: {e}")

def start_web_server(port=8080):
    """Web sunucusunu başlat"""
    try:
        server = HTTPServer(('localhost', port), RescueWebViewer)
        url = f"http://localhost:{port}"

        print(f"🌐 Web Viewer başlatıldı!")
        print(f"📱 Adres: {url}")
        print(f"🔄 Sayfa otomatik olarak yenileniyor...")
        print(f"⏹️  Durdurmak için Ctrl+C basın")
        print(f"🚀 Tarayıcı 2 saniye sonra otomatik açılacak...")

        # Tarayıcıyı ayrı thread'de aç
        browser_thread = threading.Thread(target=open_browser, args=(url,))
        browser_thread.daemon = True
        browser_thread.start()

        server.serve_forever()
    except KeyboardInterrupt:
        print("\n👋 Web sunucu durduruluyor...")
        server.shutdown()
    except OSError as e:
        if "Address already in use" in str(e):
            print(f"❌ Port {port} zaten kullanımda. Başka bir port deneyin.")
            start_web_server(port + 1)
        else:
            print(f"❌ Sunucu hatası: {e}")


if __name__ == '__main__':
    start_web_server()