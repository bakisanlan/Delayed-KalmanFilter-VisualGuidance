#!/usr/bin/env python3
"""
ESKF Log Viewer - Professional GUI for viewing ESKF logs and plotting CSV data

Features:
- Terminal log viewer with ANSI color rendering
- CSV state plotter with matplotlib subplots
- Search functionality
- Dark theme

Usage:
    python3 eskf_log_viewer.py
"""

import sys
import os
import re
import glob
from pathlib import Path

# PyQt5 imports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QTextEdit, QLineEdit, QPushButton, QComboBox,
    QLabel, QSplitter, QFileDialog, QScrollArea, QFrame, QSizePolicy,
    QCheckBox
)
from PyQt5.QtGui import QFont, QTextCursor, QColor, QTextCharFormat, QPalette
from PyQt5.QtCore import Qt

# Matplotlib imports for plotting
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

# Pandas for CSV handling
import pandas as pd
import numpy as np

# Default paths
LOG_DIR = "/home/ituarc/ros2_ws/src/eskf_cpp_reduced/log"
CSV_DIR = "/home/ituarc/ros2_ws/src/eskf_cpp_reduced/log"
RADAR_CSV_DIR = "/home/ituarc/Documents/GitHub/Delayed-KalmanFilter-VisualGuidance/eskf_py/interceptor_sensor_emulators/log/radar"
CAMERA_CSV_DIR = "/home/ituarc/Documents/GitHub/Delayed-KalmanFilter-VisualGuidance/eskf_py/interceptor_sensor_emulators/log/camera"

# Try to load paths from config
try:
    import yaml
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'eskf_reduced_params.yaml')
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
        if 'logging' in config:
            logging_cfg = config['logging']
            if 'log_dir' in logging_cfg:
                LOG_DIR = logging_cfg['log_dir']
                CSV_DIR = logging_cfg['log_dir']
            if 'radar_csv_dir' in logging_cfg:
                RADAR_CSV_DIR = logging_cfg['radar_csv_dir']
            if 'camera_csv_dir' in logging_cfg:
                CAMERA_CSV_DIR = logging_cfg['camera_csv_dir']
except Exception as e:
    print(f"Warning: Could not load config file for paths: {e}")

# ANSI color codes to Qt colors
ANSI_COLORS = {
    '30': '#1a1a1a',  # Black
    '31': '#ff6b6b',  # Red
    '32': '#69db7c',  # Green
    '33': '#ffd43b',  # Yellow
    '34': '#74c0fc',  # Blue
    '35': '#da77f2',  # Magenta
    '36': '#66d9ef',  # Cyan
    '37': '#f8f8f2',  # White
    '90': '#6c6c6c',  # Bright Black
    '91': '#ff8787',  # Bright Red
    '92': '#8ce99a',  # Bright Green
    '93': '#ffe066',  # Bright Yellow
    '94': '#a5d8ff',  # Bright Blue
    '95': '#e599f7',  # Bright Magenta
    '96': '#99f6e4',  # Bright Cyan
    '97': '#ffffff',  # Bright White
}

DARK_STYLE = """
QMainWindow, QWidget {
    background-color: #1e1e2e;
    color: #cdd6f4;
}
QTabWidget::pane {
    border: 1px solid #45475a;
    background-color: #1e1e2e;
}
QTabBar::tab {
    background-color: #313244;
    color: #cdd6f4;
    padding: 10px 20px;
    border: 1px solid #45475a;
    border-bottom: none;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
}
QTabBar::tab:selected {
    background-color: #45475a;
    color: #f5c2e7;
}
QTextEdit {
    background-color: #11111b;
    color: #cdd6f4;
    border: 1px solid #45475a;
    border-radius: 6px;
    font-family: 'JetBrains Mono', 'Fira Code', 'Consolas', monospace;
    font-size: 12px;
    padding: 8px;
}
QLineEdit {
    background-color: #313244;
    color: #cdd6f4;
    border: 1px solid #45475a;
    border-radius: 6px;
    padding: 8px 12px;
    font-size: 13px;
}
QLineEdit:focus {
    border: 2px solid #89b4fa;
}
QPushButton {
    background-color: #89b4fa;
    color: #1e1e2e;
    border: none;
    border-radius: 6px;
    padding: 10px 20px;
    font-weight: bold;
    font-size: 13px;
}
QPushButton:hover {
    background-color: #b4befe;
}
QPushButton:pressed {
    background-color: #74c7ec;
}
QComboBox {
    background-color: #313244;
    color: #cdd6f4;
    border: 1px solid #45475a;
    border-radius: 6px;
    padding: 8px 12px;
    font-size: 13px;
    min-width: 300px;
}
QComboBox:hover {
    border: 1px solid #89b4fa;
}
QComboBox::drop-down {
    border: none;
    width: 30px;
}
QComboBox QAbstractItemView {
    background-color: #313244;
    color: #cdd6f4;
    selection-background-color: #45475a;
}
QLabel {
    color: #cdd6f4;
    font-size: 13px;
}
QScrollBar:vertical {
    background-color: #1e1e2e;
    width: 12px;
    border-radius: 6px;
}
QScrollBar::handle:vertical {
    background-color: #45475a;
    border-radius: 6px;
    min-height: 30px;
}
QScrollBar::handle:vertical:hover {
    background-color: #585b70;
}
"""


class AnsiParser:
    """Parse ANSI escape codes and convert to HTML"""
    
    ANSI_PATTERN = re.compile(r'\x1b\[([0-9;]*)m')
    
    @classmethod
    def ansi_to_html(cls, text: str) -> str:
        """Convert text with ANSI codes to HTML with color spans"""
        result = []
        current_style = {}
        last_end = 0
        
        for match in cls.ANSI_PATTERN.finditer(text):
            # Add text before this escape sequence
            before_text = text[last_end:match.start()]
            if before_text:
                result.append(cls._apply_style(cls._escape_html(before_text), current_style))
            
            # Parse the escape codes
            codes = match.group(1).split(';') if match.group(1) else ['0']
            for code in codes:
                if code == '0' or code == '':
                    current_style = {}
                elif code == '1':
                    current_style['bold'] = True
                elif code in ANSI_COLORS:
                    current_style['color'] = ANSI_COLORS[code]
            
            last_end = match.end()
        
        # Add remaining text
        remaining = text[last_end:]
        if remaining:
            result.append(cls._apply_style(cls._escape_html(remaining), current_style))
        
        return ''.join(result)
    
    @staticmethod
    def _escape_html(text: str) -> str:
        """Escape HTML special characters"""
        return (text.replace('&', '&amp;')
                    .replace('<', '&lt;')
                    .replace('>', '&gt;')
                    .replace('"', '&quot;'))
    
    @staticmethod
    def _apply_style(text: str, style: dict) -> str:
        """Wrap text in HTML span with style"""
        if not style:
            return text
        
        css = []
        if 'color' in style:
            css.append(f"color: {style['color']}")
        if style.get('bold'):
            css.append("font-weight: bold")
        
        if css:
            return f'<span style="{"; ".join(css)}">{text}</span>'
        return text


def find_matching_sensor_log(eskf_csv_path: str, sensor_dir: str, tolerance_sec: int = 60) -> str:
    """Find the corresponding radar or camera log based on timestamp."""
    csv_basename = os.path.basename(eskf_csv_path)
    csv_match = re.search(r'(\d{8}_\d{6})\.csv', csv_basename)
    if not csv_match:
        return ""
    
    csv_datetime_str = csv_match.group(1)
    try:
        csv_dt = pd.to_datetime(csv_datetime_str, format='%Y%m%d_%H%M%S')
    except:
        return ""
    
    if not os.path.exists(sensor_dir):
        return ""
    
    sensor_logs = glob.glob(os.path.join(sensor_dir, "*.csv"))
    
    best_match = ""
    best_diff = float('inf')
    
    for log in sensor_logs:
        match = re.search(r'(\d{8}_\d{6})\.csv', os.path.basename(log))
        if match:
            try:
                dt = pd.to_datetime(match.group(1), format='%Y%m%d_%H%M%S')
                diff = abs((csv_dt - dt).total_seconds())
                if diff <= tolerance_sec and diff < best_diff:
                    best_diff = diff
                    best_match = log
            except:
                continue
    
    return best_match


class LogViewerTab(QWidget):
    """Tab for viewing terminal logs with ANSI colors and search"""
    
    def __init__(self):
        super().__init__()
        self.current_lines = []
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(12)
        layout.setContentsMargins(16, 16, 16, 16)
        
        # Header with file selector
        header = QHBoxLayout()
        header.addWidget(QLabel("📁 Log File:"))
        
        self.file_combo = QComboBox()
        self.file_combo.currentTextChanged.connect(self.load_file)
        header.addWidget(self.file_combo, 1)
        
        refresh_btn = QPushButton("🔄 Refresh")
        refresh_btn.clicked.connect(self.refresh_files)
        header.addWidget(refresh_btn)
        
        layout.addLayout(header)
        
        # Search bar
        search_layout = QHBoxLayout()
        search_layout.addWidget(QLabel("🔍 Search:"))
        
        self.search_input = QLineEdit()
        self.search_input.setPlaceholderText("Type to filter lines...")
        self.search_input.textChanged.connect(self.filter_lines)
        search_layout.addWidget(self.search_input, 1)
        
        layout.addLayout(search_layout)
        
        # Log display
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setFont(QFont('JetBrains Mono', 11))
        layout.addWidget(self.log_display, 1)
        
        # Status bar
        self.status_label = QLabel("No file loaded")
        self.status_label.setStyleSheet("color: #6c7086; font-size: 11px;")
        layout.addWidget(self.status_label)
        
        # Initial file load
        self.refresh_files()
    
    def refresh_files(self):
        """Refresh the list of log files"""
        self.file_combo.clear()
        
        if os.path.exists(LOG_DIR):
            files = sorted(glob.glob(os.path.join(LOG_DIR, "*.log")), reverse=True)
            for f in files:
                self.file_combo.addItem(os.path.basename(f), f)
        
        if self.file_combo.count() == 0:
            self.file_combo.addItem("No log files found")
    
    def load_file(self, filename):
        """Load and display a log file"""
        if not filename or filename.startswith("No "):
            return
        
        filepath = self.file_combo.currentData()
        if not filepath or not os.path.exists(filepath):
            return
        
        try:
            with open(filepath, 'r', encoding='utf-8', errors='replace') as f:
                self.current_lines = f.readlines()
            
            self.display_lines(self.current_lines)
            self.status_label.setText(f"Loaded: {filename} ({len(self.current_lines)} lines)")
        except Exception as e:
            self.log_display.setHtml(f'<span style="color: #f38ba8;">Error loading file: {e}</span>')
            self.status_label.setText(f"Error: {e}")
    
    def display_lines(self, lines):
        """Display lines with ANSI color parsing"""
        html_parts = ['<pre style="margin: 0; white-space: pre-wrap;">']
        
        for line in lines:
            html_line = AnsiParser.ansi_to_html(line.rstrip('\n'))
            html_parts.append(html_line + '<br>')
        
        html_parts.append('</pre>')
        self.log_display.setHtml(''.join(html_parts))
    
    def filter_lines(self, search_text):
        """Filter lines based on search text"""
        if not self.current_lines:
            return
        
        if not search_text:
            self.display_lines(self.current_lines)
            return
        
        search_lower = search_text.lower()
        filtered = [line for line in self.current_lines 
                   if search_lower in line.lower()]
        
        self.display_lines(filtered)
        self.status_label.setText(f"Showing {len(filtered)} of {len(self.current_lines)} lines")


class StatePlotterTab(QWidget):
    """Tab for plotting CSV state data"""
    
    def __init__(self):
        super().__init__()
        self.df = None
        self.radar_df = None
        self.camera_df = None
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(12)
        layout.setContentsMargins(16, 16, 16, 16)
        
        # Header with file selector
        header = QHBoxLayout()
        header.addWidget(QLabel("📊 CSV File:"))
        
        self.file_combo = QComboBox()
        self.file_combo.currentTextChanged.connect(self.load_and_plot)
        header.addWidget(self.file_combo, 1)
        
        refresh_btn = QPushButton("🔄 Refresh")
        refresh_btn.clicked.connect(self.refresh_files)
        header.addWidget(refresh_btn)
        
        layout.addLayout(header)
        
        # Status labels for radar/camera
        status_layout = QHBoxLayout()
        self.radar_status = QLabel("Radar: No log")
        self.radar_status.setStyleSheet("color: #6c7086; font-size: 11px;")
        status_layout.addWidget(self.radar_status)
        
        self.camera_status = QLabel("Camera: No log")
        self.camera_status.setStyleSheet("color: #6c7086; font-size: 11px;")
        status_layout.addWidget(self.camera_status)
        status_layout.addStretch()
        
        layout.addLayout(status_layout)
        
        # State selection checkboxes
        state_select_layout = QHBoxLayout()
        state_select_layout.addWidget(QLabel("📈 Show States:"))
        
        # Define state groups with their display names
        self.state_groups = [
            ('position', 'Target Position'),
            ('velocity', 'Target Velocity'),
            ('pbar', 'Pbar'),
        ]
        
        self.state_checkboxes = {}
        checkbox_style = """
            QCheckBox { color: #cdd6f4; font-size: 11px; margin-right: 8px; }
            QCheckBox::indicator { width: 14px; height: 14px; }
            QCheckBox::indicator:checked { background-color: #89b4fa; border: 2px solid #89b4fa; border-radius: 3px; }
            QCheckBox::indicator:unchecked { background-color: #313244; border: 2px solid #45475a; border-radius: 3px; }
        """
        
        # States enabled by default
        default_enabled = {'position', 'velocity', 'pbar'}
        
        for key, label in self.state_groups:
            cb = QCheckBox(label)
            cb.setChecked(key in default_enabled)
            cb.setStyleSheet(checkbox_style)
            cb.stateChanged.connect(self.on_state_selection_changed)
            self.state_checkboxes[key] = cb
            state_select_layout.addWidget(cb)
        
        state_select_layout.addStretch()
        layout.addLayout(state_select_layout)
        
        # Matplotlib figure with scroll area
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setStyleSheet("QScrollArea { border: none; background-color: #1e1e2e; }")
        
        # Container for the plot (inside scroll area)
        self.plot_container = QWidget()
        self.plot_layout = QVBoxLayout(self.plot_container)
        
        # Create figure with subplots (initial size, will be adjusted dynamically)
        self.figure = Figure(figsize=(14, 24), dpi=100, facecolor='#1e1e2e')
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setMinimumHeight(2000)
        
        # Navigation toolbar - placed OUTSIDE scroll area to stay visible
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.toolbar.setStyleSheet("""
            QToolBar { background-color: #313244; border: none; padding: 4px; }
            QToolButton { background-color: #313244; color: #cdd6f4; }
        """)
        layout.addWidget(self.toolbar)  # Add to main layout, not scroll area
        
        # Only canvas goes in the scroll area
        self.plot_layout.addWidget(self.canvas)
        
        self.scroll_area.setWidget(self.plot_container)
        layout.addWidget(self.scroll_area, 1)
        
        # Status bar
        self.status_label = QLabel("No file loaded")
        self.status_label.setStyleSheet("color: #6c7086; font-size: 11px;")
        layout.addWidget(self.status_label)
        
        # Initial file load
        self.refresh_files()
    
    def refresh_files(self):
        """Refresh the list of CSV files"""
        self.file_combo.clear()
        
        if os.path.exists(CSV_DIR):
            files = sorted(glob.glob(os.path.join(CSV_DIR, "*.csv")), reverse=True)
            # Filter out small/empty files
            files = [f for f in files if os.path.getsize(f) > 1000]
            for f in files:
                size_mb = os.path.getsize(f) / (1024 * 1024)
                self.file_combo.addItem(f"{os.path.basename(f)} ({size_mb:.1f} MB)", f)
        
        if self.file_combo.count() == 0:
            self.file_combo.addItem("No CSV files found")
    
    def load_and_plot(self, filename):
        """Load CSV and create plots"""
        if not filename or filename.startswith("No "):
            return
        
        filepath = self.file_combo.currentData()
        if not filepath or not os.path.exists(filepath):
            return
        
        try:
            self.status_label.setText("Loading CSV data...")
            QApplication.processEvents()
            
            self.df = pd.read_csv(filepath)
            
            # Convert timestamp to relative time
            if 'timestamp' in self.df.columns:
                self.df['time'] = self.df['timestamp'] - self.df['timestamp'].iloc[0]
                self.eskf_t0 = self.df['timestamp'].iloc[0]  # Store for PR alignment
            else:
                self.df['time'] = np.arange(len(self.df)) * 0.005  # 200Hz default
                self.eskf_t0 = 0
            
            # Try to find and load matching sensor logs
            self.radar_df = None
            self.camera_df = None
            
            radar_path = find_matching_sensor_log(filepath, RADAR_CSV_DIR)
            if radar_path:
                self.radar_df = pd.read_csv(radar_path)
                if 'timestamp' in self.radar_df.columns:
                    self.radar_df['time'] = self.radar_df['timestamp'] - self.eskf_t0
                self.radar_status.setText(f"Radar: ✓ {os.path.basename(radar_path)}")
                self.radar_status.setStyleSheet("color: #a6e3a1; font-size: 11px;")
            else:
                self.radar_status.setText("Radar: No log found")
                self.radar_status.setStyleSheet("color: #f9e2af; font-size: 11px;")
                
            camera_path = find_matching_sensor_log(filepath, CAMERA_CSV_DIR)
            if camera_path:
                self.camera_df = pd.read_csv(camera_path)
                if 'timestamp' in self.camera_df.columns:
                    self.camera_df['time'] = self.camera_df['timestamp'] - self.eskf_t0
                self.camera_status.setText(f"Camera: ✓ {os.path.basename(camera_path)}")
                self.camera_status.setStyleSheet("color: #a6e3a1; font-size: 11px;")
            else:
                self.camera_status.setText("Camera: No log found")
                self.camera_status.setStyleSheet("color: #f9e2af; font-size: 11px;")
            
            self.create_plots()
            self.status_label.setText(f"Loaded: {os.path.basename(filepath)} ({len(self.df)} samples)")
            
        except Exception as e:
            self.status_label.setText(f"Error: {e}")
            import traceback
            traceback.print_exc()
    
    # PR checkbox removed
    
    def on_state_selection_changed(self, state):
        """Handle state selection checkbox toggle"""
        if self.df is not None:
            self.create_plots()
    
    def get_selected_states(self):
        """Return list of selected state group keys"""
        return [key for key, cb in self.state_checkboxes.items() if cb.isChecked()]
    
    def create_plots(self):
        """Create all state and covariance subplots for selected states"""
        if self.df is None:
            return
        
        self.figure.clear()
        
        # Set dark style for plots
        plt.style.use('dark_background')
        
        selected_keys = self.get_selected_states()
        if not selected_keys:
            self.canvas.setMinimumHeight(200)
            ax = self.figure.add_subplot(1, 1, 1)
            ax.text(0.5, 0.5, 'Select at least one state to display', 
                   ha='center', va='center', fontsize=14, color='#6c7086')
            ax.set_facecolor('#11111b')
            ax.axis('off')
            self.figure.tight_layout()
            self.canvas.draw()
            return
            
        axes_defs = []
        if 'position' in selected_keys:
            axes_defs.append(('Target Position X/N', 'Pos (m)', self.radar_df, 'truth_pos_n', self.radar_df, 'meas_pos_n', self.df, 'pt_x', 'P_pt_x'))
            axes_defs.append(('Target Position Y/E', 'Pos (m)', self.radar_df, 'truth_pos_e', self.radar_df, 'meas_pos_e', self.df, 'pt_y', 'P_pt_y'))
            axes_defs.append(('Target Position Z/D', 'Pos (m)', self.radar_df, 'truth_pos_d', self.radar_df, 'meas_pos_d', self.df, 'pt_z', 'P_pt_z'))
        if 'velocity' in selected_keys:
            axes_defs.append(('Target Velocity X/N', 'Vel (m/s)', self.radar_df, 'truth_vel_n', self.radar_df, 'meas_vel_n', self.df, 'vt_x', 'P_vt_x'))
            axes_defs.append(('Target Velocity Y/E', 'Vel (m/s)', self.radar_df, 'truth_vel_e', self.radar_df, 'meas_vel_e', self.df, 'vt_y', 'P_vt_y'))
            axes_defs.append(('Target Velocity Z/D', 'Vel (m/s)', self.radar_df, 'truth_vel_d', self.radar_df, 'meas_vel_d', self.df, 'vt_z', 'P_vt_z'))
        if 'pbar' in selected_keys:
            axes_defs.append(('Target Pbar X', 'Normalized', self.camera_df, 'true_pbar_x', self.camera_df, 'meas_pbar_x', self.df, 'pbar_x', 'P_pbar_x'))
            axes_defs.append(('Target Pbar Y', 'Normalized', self.camera_df, 'true_pbar_y', self.camera_df, 'meas_pbar_y', self.df, 'pbar_y', 'P_pbar_y'))

        n_subplots = len(axes_defs)
        
        # Determine layout (we'll make it 1 column, dynamic height)
        # Give each subplot about 250px height
        row_height = 250
        total_height = n_subplots * row_height
        self.canvas.setMinimumHeight(total_height)
        self.figure.set_size_inches(14, total_height / 100)  # dpi=100
        
        for idx, (title, ylabel, tr_df, tr_col, ms_df, ms_col, est_df, est_col, cov_col) in enumerate(axes_defs):
            ax = self.figure.add_subplot(n_subplots, 1, idx + 1)
            
            # Plot True
            if tr_df is not None and tr_col in tr_df.columns:
                ax.plot(tr_df['time'], tr_df[tr_col], color='#89b4fa', label='True', linewidth=1.5)
                
            # Plot Measured
            if ms_df is not None and ms_col in ms_df.columns:
                 ax.plot(ms_df['time'], ms_df[ms_col], color='#fab387', label='Measured', marker='.', linestyle='None', markersize=3, alpha=0.6)
                 
            # Plot Estimated
            if est_df is not None and est_col in est_df.columns:
                ax.plot(est_df['time'], est_df[est_col], color='#a6e3a1', label='Estimated', linewidth=1.5)
                
                # Plot Covariance Bounds
                if cov_col in est_df.columns:
                    var = est_df[cov_col]
                    var_safe = np.maximum(var, 0.0)
                    std_dev = np.sqrt(var_safe)
                    
                    valid = ~est_df[est_col].isna() & ~std_dev.isna()
                    if valid.any():
                        time_valid = est_df['time'][valid]
                        upper = (est_df[est_col] + 3*std_dev)[valid]
                        lower = (est_df[est_col] - 3*std_dev)[valid]
                        ax.fill_between(time_valid, lower, upper, color='#a6e3a1', alpha=0.2, label='±3σ Bound')
                                    
            ax.set_title(title, color='#cdd6f4', fontsize=11, fontweight='bold')
            ax.set_ylabel(ylabel, color='#6c7086', fontsize=9)
            if idx == n_subplots - 1:
                ax.set_xlabel('Time (s)', color='#6c7086', fontsize=9)
            ax.legend(loc='upper right', fontsize=8, framealpha=0.8)
            ax.grid(True, alpha=0.2, color='#45475a')
            ax.set_facecolor('#11111b')
            ax.tick_params(colors='#6c7086', labelsize=8)
            for spine in ax.spines.values():
                spine.set_color('#45475a')
                
        self.figure.tight_layout(pad=2.0)
        self.canvas.draw()


class ESKFLogViewer(QMainWindow):
    """Main application window"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESKF Log Viewer")
        self.setMinimumSize(1400, 900)
        self.init_ui()
    
    def init_ui(self):
        # Central widget with tabs
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Tab widget
        tabs = QTabWidget()
        tabs.addTab(LogViewerTab(), "📜 Log Viewer")
        tabs.addTab(StatePlotterTab(), "📊 State Plotter")
        
        layout.addWidget(tabs)


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setStyleSheet(DARK_STYLE)
    
    # Set dark palette
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor('#1e1e2e'))
    palette.setColor(QPalette.WindowText, QColor('#cdd6f4'))
    palette.setColor(QPalette.Base, QColor('#11111b'))
    palette.setColor(QPalette.AlternateBase, QColor('#313244'))
    palette.setColor(QPalette.Text, QColor('#cdd6f4'))
    palette.setColor(QPalette.Button, QColor('#313244'))
    palette.setColor(QPalette.ButtonText, QColor('#cdd6f4'))
    palette.setColor(QPalette.Highlight, QColor('#89b4fa'))
    palette.setColor(QPalette.HighlightedText, QColor('#1e1e2e'))
    app.setPalette(palette)
    
    window = ESKFLogViewer()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
