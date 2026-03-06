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
LOG_DIR = "//home/ituarc/ros2_ws/src/eskf_cpp/log/service_log"
CSV_DIR = "/home/ituarc/ros2_ws/src/eskf_cpp/log"

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


def parse_relative_pos_log(filepath: str) -> pd.DataFrame:
    """Parse relative_pos*.log file and extract timestamp + NED values.
    
    Log format: [INFO] [timestamp.nsec] [node]: Relative NED: N=x.xxm, E=x.xxm, D=x.xxm
    
    Returns DataFrame with columns: timestamp, pr_n, pr_e, pr_d
    """
    pattern = re.compile(
        r'\[INFO\] \[(\d+\.\d+)\] \[relative_position_node\]: Relative NED: '
        r'N=([-\d.]+)m, E=([-\d.]+)m, D=([-\d.]+)m'
    )
    
    timestamps = []
    n_vals = []
    e_vals = []
    d_vals = []
    
    try:
        with open(filepath, 'r', encoding='utf-8', errors='replace') as f:
            for line in f:
                match = pattern.search(line)
                if match:
                    timestamps.append(float(match.group(1)))
                    n_vals.append(float(match.group(2)))
                    e_vals.append(float(match.group(3)))
                    d_vals.append(float(match.group(4)))
    except Exception as e:
        print(f"Error parsing PR log: {e}")
        return pd.DataFrame()
    
    if not timestamps:
        return pd.DataFrame()
    
    return pd.DataFrame({
        'timestamp': timestamps,
        'pr_n': n_vals,
        'pr_e': e_vals,
        'pr_d': d_vals
    })


def find_matching_pr_log(eskf_csv_path: str, tolerance_sec: int = 2) -> str:
    """Find the corresponding relative_pos log file for an ESKF CSV.
    
    Matches based on filename timestamps within ±tolerance_sec.
    Returns path to matching PR log or empty string if not found.
    """
    # Extract datetime from ESKF CSV filename: eskf_YYYYMMDD_HHMMSS.csv
    csv_basename = os.path.basename(eskf_csv_path)
    csv_match = re.search(r'eskf_(\d{8}_\d{6})\.csv', csv_basename)
    if not csv_match:
        return ""
    
    csv_datetime_str = csv_match.group(1)
    
    # Parse to datetime for comparison
    try:
        csv_dt = pd.to_datetime(csv_datetime_str, format='%Y%m%d_%H%M%S')
    except:
        return ""
    
    # Search for relative_pos logs in service_log directory
    service_log_dir = os.path.join(os.path.dirname(eskf_csv_path), "service_log")
    if not os.path.exists(service_log_dir):
        return ""
    
    pr_logs = glob.glob(os.path.join(service_log_dir, "relative_pos_*.log"))
    
    best_match = ""
    best_diff = float('inf')
    
    for pr_log in pr_logs:
        pr_match = re.search(r'relative_pos_(\d{8}_\d{6})\.log', os.path.basename(pr_log))
        if pr_match:
            try:
                pr_dt = pd.to_datetime(pr_match.group(1), format='%Y%m%d_%H%M%S')
                diff = abs((csv_dt - pr_dt).total_seconds())
                if diff <= tolerance_sec and diff < best_diff:
                    best_diff = diff
                    best_match = pr_log
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
            files = sorted(glob.glob(os.path.join(LOG_DIR, "eskf_*.log")), reverse=True)
            for f in files:
                self.file_combo.addItem(os.path.basename(f), f)
        
        if self.file_combo.count() == 0:
            self.file_combo.addItem("No eskf_*.log files found")
    
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
        self.pr_df = None  # PR measurement data
        self.pr_log_path = ""  # Path to currently loaded PR log
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
        
        # PR overlay checkbox
        pr_layout = QHBoxLayout()
        self.pr_checkbox = QCheckBox("📍 Show PR Measurement (NED)")
        self.pr_checkbox.setChecked(True)
        self.pr_checkbox.setStyleSheet("""
            QCheckBox { color: #f9e2af; font-size: 12px; }
            QCheckBox::indicator { width: 16px; height: 16px; }
            QCheckBox::indicator:checked { background-color: #a6e3a1; border: 2px solid #a6e3a1; border-radius: 4px; }
            QCheckBox::indicator:unchecked { background-color: #313244; border: 2px solid #45475a; border-radius: 4px; }
        """)
        self.pr_checkbox.stateChanged.connect(self.on_pr_checkbox_changed)
        pr_layout.addWidget(self.pr_checkbox)
        
        self.pr_status = QLabel("No PR data")
        self.pr_status.setStyleSheet("color: #6c7086; font-size: 11px;")
        pr_layout.addWidget(self.pr_status)
        pr_layout.addStretch()
        
        layout.addLayout(pr_layout)
        
        # State selection checkboxes
        state_select_layout = QHBoxLayout()
        state_select_layout.addWidget(QLabel("📈 Show States:"))
        
        # Define state groups with their display names
        self.state_groups = [
            ('attitude', 'Attitude'),
            ('position', 'Position'),
            ('velocity', 'Velocity'),
            ('pbar', 'Pbar'),
            ('gyro_bias', 'Gyro Bias'),
            ('accel_bias', 'Accel Bias'),
            ('mag_bias', 'Mag Bias'),
        ]
        
        self.state_checkboxes = {}
        checkbox_style = """
            QCheckBox { color: #cdd6f4; font-size: 11px; margin-right: 8px; }
            QCheckBox::indicator { width: 14px; height: 14px; }
            QCheckBox::indicator:checked { background-color: #89b4fa; border: 2px solid #89b4fa; border-radius: 3px; }
            QCheckBox::indicator:unchecked { background-color: #313244; border: 2px solid #45475a; border-radius: 3px; }
        """
        
        # States enabled by default
        default_enabled = {'attitude', 'position', 'velocity'}
        
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
            files = sorted(glob.glob(os.path.join(CSV_DIR, "eskf_*.csv")), reverse=True)
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
            
            # Try to find and load matching PR log
            self.pr_df = None
            self.pr_log_path = find_matching_pr_log(filepath)
            
            if self.pr_log_path:
                self.pr_df = parse_relative_pos_log(self.pr_log_path)
                if not self.pr_df.empty:
                    # Convert PR timestamps to relative time aligned with ESKF
                    self.pr_df['time'] = self.pr_df['timestamp'] - self.eskf_t0
                    pr_basename = os.path.basename(self.pr_log_path)
                    self.pr_status.setText(f"✓ {pr_basename} ({len(self.pr_df)} pts)")
                    self.pr_status.setStyleSheet("color: #a6e3a1; font-size: 11px;")
                else:
                    self.pr_status.setText("PR log found but no data parsed")
                    self.pr_status.setStyleSheet("color: #f9e2af; font-size: 11px;")
            else:
                self.pr_status.setText("No matching PR log found")
                self.pr_status.setStyleSheet("color: #6c7086; font-size: 11px;")
            
            self.create_plots()
            self.status_label.setText(f"Loaded: {os.path.basename(filepath)} ({len(self.df)} samples)")
            
        except Exception as e:
            self.status_label.setText(f"Error: {e}")
            import traceback
            traceback.print_exc()
    
    def on_pr_checkbox_changed(self, state):
        """Handle PR overlay checkbox toggle"""
        if self.df is not None:
            self.create_plots()
    
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
        
        # Colors for the three axes
        colors = ['#f38ba8', '#a6e3a1', '#89b4fa']  # Red, Green, Blue
        cov_colors = ['#f38ba8', '#a6e3a1', '#89b4fa']
        
        time = self.df['time']
        
        # Define all plot groups: (key, title, state_cols, cov_cols, state_unit, cov_unit)
        all_plot_groups = [
            ('attitude', 'Attitude', ['roll_deg', 'pitch_deg', 'yaw_deg'], 
             ['P_dtheta_x', 'P_dtheta_y', 'P_dtheta_z'], '(deg)', '(rad²)'),
            ('position', 'Position', ['x', 'y', 'z'], 
             ['P_dpos_x', 'P_dpos_y', 'P_dpos_z'], '(m)', '(m²)'),
            ('velocity', 'Velocity', ['vx', 'vy', 'vz'], 
             ['P_dvel_x', 'P_dvel_y', 'P_dvel_z'], '(m/s)', '(m²/s²)'),
            ('pbar', 'Pbar', ['pbar_x', 'pbar_y'], 
             ['P_dpbar_x', 'P_dpbar_y'], '', ''),
            ('gyro_bias', 'Gyro Bias', ['bgyr_x', 'bgyr_y', 'bgyr_z'], 
             ['P_dbgyr_x', 'P_dbgyr_y', 'P_dbgyr_z'], '(rad/s)', ''),
            ('accel_bias', 'Accel Bias', ['bacc_x', 'bacc_y', 'bacc_z'], 
             ['P_dbacc_x', 'P_dbacc_y', 'P_dbacc_z'], '(m/s²)', ''),
            ('mag_bias', 'Mag Bias', ['bmag_x', 'bmag_y', 'bmag_z'], 
             ['P_dbmag_x', 'P_dbmag_y', 'P_dbmag_z'], '', ''),
        ]
        
        # Filter to selected states only
        selected_keys = self.get_selected_states()
        plot_groups = [pg for pg in all_plot_groups if pg[0] in selected_keys]
        
        n_groups = len(plot_groups)
        
        if n_groups == 0:
            # No states selected - show message
            self.canvas.setMinimumHeight(200)
            ax = self.figure.add_subplot(1, 1, 1)
            ax.text(0.5, 0.5, 'Select at least one state to display', 
                   ha='center', va='center', fontsize=14, color='#6c7086')
            ax.set_facecolor('#11111b')
            ax.axis('off')
            self.figure.tight_layout()
            self.canvas.draw()
            return
        
        # Dynamic figure sizing based on selection count
        # 1 state: ~500px height (fills view nicely)
        # 2 states: ~400px each (fits without scrolling)
        # 3+ states: ~300px each (requires scrolling)
        if n_groups == 1:
            row_height = 500
        elif n_groups == 2:
            row_height = 400
        else:
            row_height = 300
        
        total_height = n_groups * row_height
        self.canvas.setMinimumHeight(total_height)
        self.figure.set_size_inches(14, total_height / 100)  # dpi=100
        
        for row_idx, (key, title, state_cols, cov_cols, state_unit, cov_unit) in enumerate(plot_groups):
            # State subplot
            ax1 = self.figure.add_subplot(n_groups, 2, row_idx * 2 + 1)
            for i, col in enumerate(state_cols):
                if col in self.df.columns:
                    label = col.split('_')[-1] if '_' in col else col
                    ax1.plot(time, self.df[col], color=colors[i % 3], 
                            label=label, linewidth=0.8, alpha=0.9)
            
            # Add PR overlay on Position plot if checkbox is checked and data exists
            if key == 'position' and self.pr_checkbox.isChecked() and self.pr_df is not None and not self.pr_df.empty:
                # PR colors (lighter/dashed versions): N=x, E=y, D=z mapping
                pr_colors = ['#fab387', '#94e2d5', '#b4befe']  # Orange, Teal, Lavender
                pr_cols = [('pr_n', 'N(PR)'), ('pr_e', 'E(PR)'), ('pr_d', 'D(PR)')]
                for i, (col, lbl) in enumerate(pr_cols):
                    if col in self.pr_df.columns:
                        ax1.plot(self.pr_df['time'], self.pr_df[col], 
                                color=pr_colors[i], label=lbl, 
                                linewidth=1.2, linestyle='--', alpha=0.85, marker='o', 
                                markersize=2, markevery=10)
            
            ax1.set_title(f'{title} {state_unit}', color='#cdd6f4', fontsize=11, fontweight='bold')
            ax1.set_xlabel('Time (s)', color='#6c7086', fontsize=9)
            ax1.legend(loc='upper right', fontsize=8, framealpha=0.8)
            ax1.grid(True, alpha=0.2, color='#45475a')
            ax1.set_facecolor('#11111b')
            ax1.tick_params(colors='#6c7086', labelsize=8)
            for spine in ax1.spines.values():
                spine.set_color('#45475a')
            
            # Covariance subplot
            ax2 = self.figure.add_subplot(n_groups, 2, row_idx * 2 + 2)
            for i, col in enumerate(cov_cols):
                if col in self.df.columns:
                    label = col.split('_')[-1] if '_' in col else col
                    ax2.plot(time, self.df[col], color=cov_colors[i % 3], 
                            label=f'σ²_{label}', linewidth=0.8, alpha=0.9)
            
            ax2.set_title(f'{title} Covariance {cov_unit}', color='#cdd6f4', fontsize=11, fontweight='bold')
            ax2.set_xlabel('Time (s)', color='#6c7086', fontsize=9)
            ax2.legend(loc='upper right', fontsize=8, framealpha=0.8)
            ax2.grid(True, alpha=0.2, color='#45475a')
            ax2.set_facecolor('#11111b')
            ax2.tick_params(colors='#6c7086', labelsize=8)
            ax2.set_yscale('log')  # Log scale for covariance
            for spine in ax2.spines.values():
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
