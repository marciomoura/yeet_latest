"""
Analysis Utilities for LCI Control Systems

This module provides common plotting functions and analysis tools for
current and speed control loop analysis.
"""

import numpy as np
import matplotlib.pyplot as plt
import control as ct


def setup_bode_plot_figure(title, num_subplot_pairs=1, figsize=(12, 9)):
    """
    Setup standardized Bode plot figure with magnitude and phase subplots.
    
    Args:
        title: Figure title
        num_subplot_pairs: Number of magnitude/phase subplot pairs
        figsize: Figure size tuple
        
    Returns:
        (fig, axes): Figure and axes array (2*num_subplot_pairs elements)
    """
    fig, axes = plt.subplots(2 * num_subplot_pairs, 1, figsize=figsize, sharex=True)
    
    if num_subplot_pairs == 1:
        axes = [axes[0], axes[1]]  # Ensure axes is always a list
    
    fig.suptitle(title, fontsize=16)
    return fig, axes


def setup_step_plot_figure(title, figsize=(12, 8)):
    """
    Setup standardized step response figure.
    
    Args:
        title: Figure title
        figsize: Figure size tuple
        
    Returns:
        (fig, ax): Figure and axis
    """
    fig, ax = plt.subplots(1, 1, figsize=figsize)
    ax.set_title(title, fontsize=16)
    return fig, ax


def plot_bode_response(axes, tf, label, frequency_range_hz=(0.1, 1000)):
    """
    Plot Bode magnitude and phase response for a transfer function.
    
    Args:
        axes: List of [magnitude_axis, phase_axis]
        tf: Transfer function to plot
        label: Legend label
        frequency_range_hz: Tuple of (min_freq, max_freq) in Hz
    """
    # Generate frequency vector
    f_min, f_max = frequency_range_hz
    omega = np.logspace(np.log10(2*np.pi*f_min), np.log10(2*np.pi*f_max), 1000)
    
    # Compute frequency response
    mag, phase, _ = ct.frequency_response(tf, omega)
    freq_hz = omega / (2 * np.pi)
    
    # Plot magnitude (dB scale)
    axes[0].semilogx(freq_hz, 20 * np.log10(mag), label=label)
    
    # Plot phase (degrees)
    axes[1].semilogx(freq_hz, np.rad2deg(phase), label=label)


def plot_step_response(ax, tf, label, time_span_s=None, num_points=1000):
    """
    Plot step response for a transfer function.
    
    Args:
        ax: Matplotlib axis
        tf: Transfer function
        label: Legend label
        time_span_s: Time span for simulation (auto-determined if None)
        num_points: Number of time points
    """
    if time_span_s is None:
        # Auto-determine time span based on settling characteristics
        time_span_s = 5.0  # Default 5 seconds
    
    time_vector = np.linspace(0, time_span_s, num_points)
    _, y_out = ct.step_response(tf, T=time_vector)
    
    ax.plot(time_vector, y_out, label=label)


def finalize_bode_plot(fig, axes, frequency_range_hz=(0.1, 1000)):
    """
    Apply standard formatting to Bode plot figures.
    
    Args:
        fig: Figure handle
        axes: Axes array [magnitude_axis, phase_axis]
        frequency_range_hz: Frequency range for x-axis limits
    """
    # Magnitude subplot formatting
    axes[0].set_ylabel('Magnitude (dB)')
    axes[0].grid(True, which='both', alpha=0.3)
    axes[0].set_xlim(frequency_range_hz)
    
    # Phase subplot formatting  
    axes[1].set_xlabel('Frequency (Hz)')
    axes[1].set_ylabel('Phase (deg)')
    axes[1].grid(True, which='both', alpha=0.3)
    axes[1].set_xlim(frequency_range_hz)
    
    # Add legends
    axes[0].legend(fontsize='small')
    axes[1].legend(fontsize='small')
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])


def finalize_step_plot(fig, ax):
    """
    Apply standard formatting to step response figures.
    
    Args:
        fig: Figure handle
        ax: Axis handle
    """
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Amplitude (p.u.)')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize='small')
    plt.tight_layout()


def print_controller_summary(kp, ti_s, loop_type="Current", scenario_label="", units=""):
    """
    Print standardized controller gain summary.
    
    Args:
        kp: Proportional gain
        ti_s: Integral time constant [s]
        loop_type: "Current" or "Speed"
        scenario_label: Description of the scenario
        units: Units for the proportional gain
    """
    ki = kp / ti_s
    print(f"\n{loop_type} Controller - {scenario_label}")
    print(f"  Kp = {kp:.4f} {units}")
    print(f"  Ki = {ki:.4f} {units}/s") 
    print(f"  Ti = {ti_s*1000:.1f} ms")


def generate_frequency_vector(f_min_hz=0.1, f_max_hz=1000, points_per_decade=50):
    """
    Generate logarithmically spaced frequency vector.
    
    Args:
        f_min_hz: Minimum frequency [Hz]
        f_max_hz: Maximum frequency [Hz]
        points_per_decade: Number of points per frequency decade
        
    Returns:
        omega: Angular frequency vector [rad/s]
    """
    num_decades = np.log10(f_max_hz / f_min_hz)
    num_points = int(num_decades * points_per_decade)
    
    freq_hz = np.logspace(np.log10(f_min_hz), np.log10(f_max_hz), num_points)
    return 2 * np.pi * freq_hz


def configure_plot_style():
    """Configure matplotlib style for consistent appearance."""
    try:
        plt.style.use('seaborn-v0_8-whitegrid')
    except:
        # Fallback if seaborn style not available
        plt.style.use('default')
        plt.rcParams['grid.alpha'] = 0.3
        plt.rcParams['axes.grid'] = True
