"""
Serial communication module for sending joint angles to Arduino.
Handles connection, sending theta values, and receiving feedback.
"""

import serial
import serial.tools.list_ports
import time
import math
from typing import List, Optional

class ArduinoRobotComm:
    def __init__(self, port: Optional[str] = None, baudrate: int = 115200):
        """
        Initialize Arduino communication.
        
        Args:
            port: Serial port name (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
                  If None, will try to auto-detect Arduino
            baudrate: Communication speed (default: 115200)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.is_connected = False
        
    def list_available_ports(self):
        """List all available serial ports."""
        ports = serial.tools.list_ports.comports()
        available = []
        print("\nAvailable serial ports:")
        for port in ports:
            print(f"  - {port.device}: {port.description}")
            available.append(port.device)
        return available
    
    def connect(self, timeout: float = 2.0):
        """
        Connect to Arduino.
        
        Args:
            timeout: Connection timeout in seconds
        
        Returns:
            bool: True if connected successfully
        """
        if self.port is None:
            # Try to auto-detect Arduino
            ports = serial.tools.list_ports.comports()
            for port in ports:
                if 'Arduino' in port.description or 'CH340' in port.description or 'USB' in port.description:
                    self.port = port.device
                    print(f"Auto-detected Arduino on {self.port}")
                    break
            
            if self.port is None:
                print("Could not auto-detect Arduino. Available ports:")
                self.list_available_ports()
                return False
        
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=timeout,
                write_timeout=timeout
            )
            
            # Wait for Arduino to reset (it resets when serial connection opens)
            time.sleep(2)
            
            # Clear any startup messages
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            self.is_connected = True
            print(f"Connected to Arduino on {self.port} at {self.baudrate} baud")
            return True
            
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Close the serial connection."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.is_connected = False
            print("Disconnected from Arduino")
    
    def send_thetas_radians(self, thetas: List[float]) -> bool:
        """
        Send joint angles in radians to Arduino.
        Format: "T,theta0,theta1,theta2,...\\n"
        
        Args:
            thetas: List of joint angles in radians
        
        Returns:
            bool: True if sent successfully
        """
        if not self.is_connected or not self.serial_conn:
            print("Not connected to Arduino")
            return False
        
        try:
            # Format: T,theta0,theta1,theta2,...
            message = "T," + ",".join([f"{theta:.6f}" for theta in thetas]) + "\n"
            self.serial_conn.write(message.encode('utf-8'))
            return True
        except Exception as e:
            print(f"Error sending data: {e}")
            return False
    
    def send_thetas_degrees(self, thetas_deg: List[float]) -> bool:
        """
        Send joint angles in degrees to Arduino.
        Format: "D,theta0,theta1,theta2,...\\n"
        
        Args:
            thetas_deg: List of joint angles in degrees
        
        Returns:
            bool: True if sent successfully
        """
        if not self.is_connected or not self.serial_conn:
            print("Not connected to Arduino")
            return False
        
        try:
            # Format: D,theta0,theta1,theta2,...
            message = "D," + ",".join([f"{theta:.2f}" for theta in thetas_deg]) + "\n"
            self.serial_conn.write(message.encode('utf-8'))
            return True
        except Exception as e:
            print(f"Error sending data: {e}")
            return False
    
    def read_feedback(self) -> Optional[str]:
        """
        Read feedback from Arduino (non-blocking).
        
        Returns:
            str: Message from Arduino, or None if nothing available
        """
        if not self.is_connected or not self.serial_conn:
            return None
        
        try:
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8').strip()
                return line
        except Exception as e:
            print(f"Error reading feedback: {e}")
        
        return None
    
    def __del__(self):
        """Cleanup: disconnect when object is destroyed."""
        self.disconnect()


# Example usage function
def test_communication():
    """Test the Arduino communication."""
    print("=== Arduino Robot Communication Test ===\n")
    
    # Create communication object
    comm = ArduinoRobotComm()
    
    # List available ports
    comm.list_available_ports()
    
    # Try to connect
    port = input("\nEnter port name (or press Enter for auto-detect): ").strip()
    if port:
        comm.port = port
    
    if not comm.connect():
        print("Failed to connect. Exiting.")
        return
    
    # Test sending some angles
    print("\nSending test angles...")
    test_angles_rad = [0.0, 0.5, -0.3, 1.0, 0.0, 0.0]
    test_angles_deg = [math.degrees(a) for a in test_angles_rad]
    
    print(f"Radians: {test_angles_rad}")
    print(f"Degrees: {test_angles_deg}")
    
    # Send in radians
    comm.send_thetas_radians(test_angles_rad[:3])  # Send first 3 joints
    time.sleep(0.5)
    
    # Check for feedback
    feedback = comm.read_feedback()
    if feedback:
        print(f"Arduino response: {feedback}")
    
    # Disconnect
    comm.disconnect()


if __name__ == "__main__":
    test_communication()