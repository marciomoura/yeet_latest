#!/usr/bin/env python3
"""
Example: Programmatic Usage of MCU Network Client
Demonstrates how to use the MCUClient class in your own Python scripts.
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from network_client import MCUClient


def example_basic_usage():
    """Example 1: Basic connection and streaming"""
    print("Example 1: Basic Usage")
    print("-" * 50)
    
    # Create client instance
    client = MCUClient(host='192.168.10.31', port=7000)
    
    # Connect to MCU
    if not client.connect():
        print("Failed to connect!")
        return
    
    # Start streaming
    client.start_streaming()
    
    # Wait and collect data
    print("Collecting data for 5 seconds...")
    time.sleep(5)
    
    # Stop streaming
    client.stop_streaming()
    
    # Get statistics
    stats = client.get_stats()
    print(f"Packets received: {stats['packets_received']}")
    print(f"Packets lost: {stats['packets_lost']}")
    
    # Disconnect
    client.disconnect()
    print()


def example_parameter_control():
    """Example 2: Changing parameters during streaming"""
    print("Example 2: Parameter Control")
    print("-" * 50)
    
    client = MCUClient(host='192.168.10.31', port=7000)
    
    if not client.connect():
        return
    
    # Start streaming with default parameters
    client.start_streaming()
    print("Streaming with default amplitude (1.0)...")
    time.sleep(2)
    
    # Change amplitude
    print("Changing amplitude to 2.5...")
    client.set_amplitude(2.5)
    time.sleep(2)
    
    # Change noise level
    print("Changing noise level to 0.5...")
    client.set_noise(0.5)
    time.sleep(2)
    
    # Stop and disconnect
    client.stop_streaming()
    client.disconnect()
    print()


def example_data_collection():
    """Example 3: Collect and plot data"""
    print("Example 3: Data Collection and Plotting")
    print("-" * 50)
    
    import threading
    
    client = MCUClient(host='192.168.10.31', port=7000, buffer_size=1000)
    
    if not client.connect():
        return
    
    # Start receive thread
    receive_thread = threading.Thread(target=client.receive_loop, daemon=True)
    receive_thread.start()
    
    # Start streaming
    client.start_streaming()
    
    # Collect data for 10 seconds
    print("Collecting 10 seconds of data...")
    time.sleep(10)
    
    # Stop streaming
    client.stop_streaming()
    
    # Get all data
    data = client.get_recent_data(max_points=1000)
    
    if len(data) > 0:
        # Extract values
        samples = [d['sample_count'] for d in data]
        values = [d['value'] for d in data]
        
        # Plot
        plt.figure(figsize=(12, 6))
        plt.plot(samples, values, linewidth=0.5)
        plt.xlabel('Sample Count')
        plt.ylabel('Signal Value')
        plt.title(f'MCU Signal Data ({len(data)} samples)')
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig('mcu_signal_plot.png', dpi=150)
        print(f"✓ Plot saved to mcu_signal_plot.png")
        
        # Statistics
        print(f"\nData Statistics:")
        print(f"  Samples: {len(data)}")
        print(f"  Mean: {np.mean(values):.4f}")
        print(f"  Std Dev: {np.std(values):.4f}")
        print(f"  Min: {np.min(values):.4f}")
        print(f"  Max: {np.max(values):.4f}")
    
    client.disconnect()
    print()


def example_continuous_monitoring():
    """Example 4: Continuous monitoring with callback"""
    print("Example 4: Continuous Monitoring")
    print("-" * 50)
    
    import threading
    
    client = MCUClient(host='192.168.10.31', port=7000)
    
    if not client.connect():
        return
    
    # Start receive thread
    receive_thread = threading.Thread(target=client.receive_loop, daemon=True)
    receive_thread.start()
    
    # Start streaming
    client.start_streaming()
    
    # Monitor for 10 seconds
    print("Monitoring signal (press Ctrl+C to stop)...")
    print("Time (s) | Packets RX | Packets Lost | Latest Value")
    print("-" * 60)
    
    start_time = time.time()
    try:
        while time.time() - start_time < 10:
            stats = client.get_stats()
            data = client.get_recent_data(max_points=1)
            
            latest_value = data[0]['value'] if len(data) > 0 else 0.0
            elapsed = time.time() - start_time
            
            print(f"{elapsed:7.1f}  | {stats['packets_received']:10d} | "
                  f"{stats['packets_lost']:12d} | {latest_value:12.4f}",
                  end='\r')
            
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    
    client.stop_streaming()
    client.disconnect()
    print()


def example_save_to_file():
    """Example 5: Save data to CSV file"""
    print("Example 5: Save Data to File")
    print("-" * 50)
    
    import threading
    import csv
    
    client = MCUClient(host='192.168.10.31', port=7000, buffer_size=5000)
    
    if not client.connect():
        return
    
    # Start receive thread
    receive_thread = threading.Thread(target=client.receive_loop, daemon=True)
    receive_thread.start()
    
    # Start streaming
    client.start_streaming()
    
    # Collect data
    print("Collecting data for 5 seconds...")
    time.sleep(5)
    
    # Stop streaming
    client.stop_streaming()
    
    # Get data
    data = client.get_recent_data(max_points=5000)
    
    # Save to CSV
    if len(data) > 0:
        filename = f"mcu_data_{int(time.time())}.csv"
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Packet Count', 'Timestamp (ms)', 'Sample Count', 'Signal Value'])
            for d in data:
                writer.writerow([d['packet_count'], d['timestamp'], 
                               d['sample_count'], d['value']])
        
        print(f"✓ Saved {len(data)} samples to {filename}")
    
    client.disconnect()
    print()


if __name__ == '__main__':
    print("=" * 60)
    print("MCU Network Client Examples")
    print("=" * 60)
    print()
    
    try:
        # Run examples
        example_basic_usage()
        example_parameter_control()
        example_data_collection()
        example_continuous_monitoring()
        example_save_to_file()
        
        print("=" * 60)
        print("✓ All examples completed!")
        print("=" * 60)
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
