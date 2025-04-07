import sys
import serial
import serial.tools.list_ports
import numpy as np
from PyQt5.QtGui import QMovie
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QWidget,
    QPushButton, QLabel, QFileDialog, QMessageBox,
    QComboBox, QHBoxLayout
)

from FinalSet.processing import ensure_binary_voxel_grid
from processing import stl_to_fixed_size_voxels, downsample_with_threshold
from visualization import visualize_voxel_grid_blocky, visualize_multiple_views


# Define a worker thread for STL processing
class ProcessingThread(QThread):
    finished = pyqtSignal(list)  # Signal with data result
    error = pyqtSignal(str)  # Signal for any errors

    def __init__(self, selected_file_path):
        super().__init__()
        self.selected_file_path = selected_file_path

    def run(self):
        """
        The heavy processing task runs in this thread.
        """
        try:
            target_shape_high_res = (64, 64, 64)
            target_shape_low_res = (8, 8, 8)
            threshold = 0.1

            # Process the high-resolution voxel grid
            high_res_voxel_grid = stl_to_fixed_size_voxels(
                self.selected_file_path, target_shape=target_shape_high_res, debug=True
            )

            # Create visualizations for the high-resolution grid
            visualize_voxel_grid_blocky(
                high_res_voxel_grid,
                perspective={"elev": 45, "azim": 45},
                title="64x64x64 Voxel Grid - Top View"
            )

            # Downsample to a low-resolution voxel grid
            low_res_voxel_grid = downsample_with_threshold(
                high_res_voxel_grid, target_shape=target_shape_low_res, threshold=threshold, debug=True
            )
            # low_res_voxel_grid = ensure_binary_voxel_grid(low_res_voxel_grid)
            # print("Array in binary: ", low_res_voxel_grid)

            # Visualize multiple perspectives for the low-res grid
            angles = [
                {"elev": 30, "azim": 30},
                {"elev": 60, "azim": 60},
                {"elev": 0, "azim": 90},
                {"elev": 90, "azim": 90},
            ]
            visualize_multiple_views(low_res_voxel_grid, angles=angles, title_prefix="8x8x8 Voxel Grid")

            # Processing and visualization are completed successfully
            self.finished.emit([
                "Success: STL file processed and visualized successfully!",
                low_res_voxel_grid  # Emit the array as the second element
            ])

        except Exception as e:
            # Handle any errors encountered during processing
            self.error.emit(str(e))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("STL to Voxel Grid Processor")

        # Main widget and layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        self.setup_serial_controls()

        # File selection label and button
        self.file_label = QLabel("No file selected")
        self.layout.addWidget(self.file_label)

        self.select_file_button = QPushButton("Select STL File")
        self.select_file_button.clicked.connect(self.select_file)
        self.layout.addWidget(self.select_file_button)

        # Process button
        self.process_button = QPushButton("Process STL File")
        self.process_button.clicked.connect(self.process_file)
        self.layout.addWidget(self.process_button)

        # Loading spinner (QLabel with QMovie)
        self.spinner_label = QLabel()
        self.spinner_label.setAlignment(Qt.AlignCenter)  # Center the loading spinner
        self.spinner_label.setVisible(False)  # Initially hidden
        self.layout.addWidget(self.spinner_label)

        # Set up the movie (your loading GIF)
        self.loading_spinner = QMovie("loading.gif")  # Replace with your GIF path
        self.spinner_label.setMovie(self.loading_spinner)

        # Store the selected file path
        self.selected_file_path = None

        # Thread for STL processing
        self.thread = None

        # # Serial communication initialization
        # try:
        #     self.ser = serial.Serial('COM6', 115200)  # Replace 'COM6' with your port
        # except serial.SerialException as e:
        #     QMessageBox.critical(self, "Serial Port Error", f"Unable to open serial port: {e}")
        #     sys.exit(1)  # Exit the application if serial port cannot be opened

        # "Ready to Send" label
        self.ready_label = QLabel("Data not ready")
        self.layout.addWidget(self.ready_label)

        # "Send" button
        self.send_button = QPushButton("Send")
        self.send_button.clicked.connect(self.send_data)
        self.layout.addWidget(self.send_button)

        # Start a timer to check for incoming data from STM32
        self.serial_timer = QTimer()
        self.serial_timer.timeout.connect(self.check_serial)
        self.serial_timer.start(100)  # Check every 100ms

    def select_file(self):
        """
        Open the file dialog to select an STL file.
        """
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Select STL File", "", "STL Files (*.stl);;All Files (*)"
        )
        if file_path:
            self.selected_file_path = file_path
            self.file_label.setText(f"Selected File: {file_path}")
        else:
            self.file_label.setText("No file selected")

    def process_file(self):
        """
        Process the selected STL file on a separate thread and show a loading spinner.
        """
        if not self.selected_file_path:
            QMessageBox.warning(self, "No File Selected", "Please select an STL file first.")
            return

        # Show and start the spinner
        self.spinner_label.setVisible(True)
        self.loading_spinner.start()

        # Start the processing thread
        self.thread = ProcessingThread(self.selected_file_path)
        self.thread.finished.connect(self.on_processing_success)
        self.thread.error.connect(self.on_processing_error)
        self.thread.start()

    def send_data(self):
        """
        Sends the voxel grid data to the STM32 with corrected orientation.
        Fixes the X-axis rotation issue where flat models appear vertical.
        """
        # Check if we have data to send
        if not hasattr(self, 'low_res_voxel_grid') or self.low_res_voxel_grid is None:
            QMessageBox.warning(self, "Not Ready", "Please process an STL file first")
            return

        # Check if we're connected
        if not hasattr(self, 'ser') or not self.ser or not self.ser.is_open:
            QMessageBox.warning(self, "Not Connected", "Please connect to a serial port first")
            return

        try:
            # Define protocol markers
            START_MARKER = 0xA5
            END_MARKER = 0x5A

            # Pack the 8x8x8 voxel grid into 64 bytes (8 bits per byte)
            packed_data = []

            # Traverse in order that fixes X-axis rotation issue:
            # Original X → LED X
            # Original Y → LED Z (inverted)
            # Original Z → LED Y
            for y in range(8):  # This maps to Z in LED cube
                for z in range(8):  # This maps to Y in LED cube
                    byte_value = 0
                    for x in range(8):  # X remains unchanged
                        # Access with corrected orientation (7-y inverts Y axis)
                        if self.low_res_voxel_grid[x, 7 - y, z]:
                            byte_value |= (1 << (7 - x))  # MSB first packing
                    packed_data.append(byte_value)

            # Prepare the complete message with protocol markers
            message = bytearray([START_MARKER]) + bytearray(packed_data) + bytearray([END_MARKER])

            # Send the data
            self.ser.write(message)
            print(f"Sent {len(message)} bytes: START + {len(packed_data)} data bytes + END")

            # Update UI
            self.ready_label.setText("Data sent successfully")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to send data: {e}")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to send data: {e}")

    def on_processing_success(self, result):
        """
        Handles successful completion of the processing task.
        """
        # Hide the spinner
        self.loading_spinner.stop()
        self.spinner_label.setVisible(False)

        # Show success message
        QMessageBox.information(self, "Success", result[0])

        # Update "Ready to Send" label
        self.ready_label.setText("Ready to send")

        # Store the low-resolution voxel grid for sending later
        self.low_res_voxel_grid = result[1]

    def on_processing_error(self, message):
        """
        Handles errors during the processing task.
        """
        # Hide the spinner
        self.loading_spinner.stop()
        self.spinner_label.setVisible(False)

        # Show error message
        QMessageBox.critical(self, "Error", message)

    def check_serial(self):
        """Check for incoming data from STM32"""
        try:
            if hasattr(self, 'ser') and self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                signal = self.ser.read(1)
                if signal[0] == 0xAA:  # READY_SIGNAL
                    self.ready_label.setText("Ready to send (STM32 is ready)")
        except Exception as e:
            print(f"Serial check error: {e}")

    def closeEvent(self, event):
        """Handle application close event"""
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
        event.accept()

    def setup_serial_controls(self):
        """Set up the serial port controls"""
        # Create horizontal layout for serial controls
        serial_layout = QHBoxLayout()

        # Add port selection combobox
        self.port_combo = QComboBox()
        self.refresh_ports()
        serial_layout.addWidget(QLabel("Serial Port:"))
        serial_layout.addWidget(self.port_combo)

        # Add refresh button
        refresh_button = QPushButton("Refresh")
        refresh_button.clicked.connect(self.refresh_ports)
        serial_layout.addWidget(refresh_button)

        # Add connect button
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_serial)
        serial_layout.addWidget(self.connect_button)

        # Add to main layout
        self.layout.addLayout(serial_layout)

        # Add status label
        self.serial_status = QLabel("Not connected")
        self.layout.addWidget(self.serial_status)

    def refresh_ports(self):
        """Refresh the list of available serial ports with descriptions"""
        self.port_combo.clear()
        ports = list(serial.tools.list_ports.comports())
        
        if ports:
            for port in ports:
                # Format: "COM3 - USB Serial Device"
                self.port_combo.addItem(f"{port.device} - {port.description}", port.device)
        else:
            self.port_combo.addItem("No ports available")

    def connect_serial(self):
        """Connect to the selected serial port"""
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
            self.connect_button.setText("Connect")
            self.serial_status.setText("Disconnected")
            return

        try:
            # Get the actual port name from the userData
            port = self.port_combo.currentData()
            if not port:  # Fall back to text if no userData (e.g., "No ports available")
                port = self.port_combo.currentText()
                if "No ports" in port:
                    QMessageBox.warning(self, "No Port", "No serial port selected")
                    return
                    
            self.ser = serial.Serial(port, 115200, timeout=1)
            self.connect_button.setText("Disconnect")
            self.serial_status.setText(f"Connected to {port}")
        except Exception as e:
            QMessageBox.critical(self, "Connection Error", f"Failed to connect: {e}").critical(self, "Connection Error", f"Failed to connect: {e}")


def main():
    app = QApplication(sys.argv)

    main_window = MainWindow()
    main_window.show()

    sys.exit(app.exec_())
