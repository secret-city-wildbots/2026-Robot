# WildBoard WebView Launcher

A simple Python script that opens a borderless, always-on-top webview window optimized for FRC dashboards.

## Features

- **Borderless window** - No title bar or frame
- **Always on top** - Stays above all other windows
- **Fullscreen minus exclusions** - Takes entire screen except:
  - Taskbar (automatically detected)
  - FRC DriverStation
  - FRC Dashboard (bottom 200px)
- **URL-based** - Load any webpage in the webview

## Installation

1. Install Python 3.8 or higher
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Usage

### With default URL (localhost:5800):
```bash
python webview_launcher.py
```

### With custom URL:
```bash
python webview_launcher.py http://example.com
python webview_launcher.py https://192.168.1.100:8080
python webview_launcher.py file:///C:/path/to/file.html
```

## Configuration

Edit `webview_launcher.py` to modify:
- **FRC Dashboard height**: Change the `frc_dashboard_height = 200` value (line ~60)
- **Default URL**: Change the default in `main()` function (line ~70)
- **Window title**: Change `"WildBoard"` in the `__init__` method (line ~23)

## Troubleshooting

- **Import errors**: Make sure dependencies are installed: `pip install -r requirements.txt`
- **URL not loading**: Ensure the URL is accessible and includes protocol (http://, https://, or file://)
- **Window positioning issues**: The script automatically accounts for the taskbar - adjust `frc_dashboard_height` if needed
