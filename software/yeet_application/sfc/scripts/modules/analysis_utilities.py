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
    
    # Plot phase (degrees) with unwrapping to avoid discontinuities
    phase_unwrapped = np.unwrap(phase)  # Unwrap phase to avoid ±180° jumps
    axes[1].semilogx(freq_hz, np.rad2deg(phase_unwrapped), label=label)


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
    # Add 0 dB reference line
    axes[0].axhline(y=0, color='red', linestyle='--', alpha=0.7, linewidth=1.5, label='0 dB')
    
    # Phase subplot formatting  
    axes[1].set_xlabel('Frequency (Hz)')
    axes[1].set_ylabel('Phase (deg)')
    axes[1].grid(True, which='both', alpha=0.3)
    axes[1].set_xlim(frequency_range_hz)
    # Add -180° reference line
    axes[1].axhline(y=-180, color='red', linestyle='--', alpha=0.7, linewidth=1.5, label='-180°')
    
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


def setup_comprehensive_analysis_figure(title, include_plant=True, include_step=True, figsize=(16, 10)):
    """
    Setup a comprehensive analysis figure with all necessary subplots in a unified dashboard.
    
    Args:
        title: Main figure title
        include_plant: Whether to include plant model comparison subplot
        include_step: Whether to include step response subplot
        figsize: Figure size tuple
        
    Returns:
        (fig, axes_dict): Figure and dictionary of axes with keys:
        - 'plant_mag', 'plant_phase' (if include_plant=True)
        - 'ol_mag', 'ol_phase': Open-loop Bode plots
        - 'cl_mag', 'cl_phase': Closed-loop Bode plots  
        - 'dist_mag', 'dist_phase': Disturbance rejection Bode plots
        - 'step': Step response (if include_step=True)
    """
    # Determine layout based on what plots to include
    if include_plant and include_step:
        # 3 rows: Bode plots in top 2 rows, step response in bottom row
        n_cols = 4  # Plant, OL, CL, Dist
        fig = plt.figure(figsize=figsize)
        gs = fig.add_gridspec(3, n_cols, height_ratios=[1, 1, 1.2], 
                             hspace=0.35, wspace=0.25,
                             left=0.06, right=0.94, top=0.92, bottom=0.06)
        
        # Create Bode plot subplots
        axes_dict = {
            'plant_mag': fig.add_subplot(gs[0, 0]),
            'plant_phase': fig.add_subplot(gs[1, 0]),
            'ol_mag': fig.add_subplot(gs[0, 1]),
            'ol_phase': fig.add_subplot(gs[1, 1]),
            'cl_mag': fig.add_subplot(gs[0, 2]),
            'cl_phase': fig.add_subplot(gs[1, 2]),
            'dist_mag': fig.add_subplot(gs[0, 3]),
            'dist_phase': fig.add_subplot(gs[1, 3]),
            'step': fig.add_subplot(gs[2, :])  # Step response spans all columns
        }
        
    elif include_plant and not include_step:
        # 2 rows, 4 columns: Plant, OL, CL, Dist
        fig = plt.figure(figsize=(figsize[0], figsize[1]*0.6))
        gs = fig.add_gridspec(2, 4, hspace=0.35, wspace=0.25,
                             left=0.06, right=0.94, top=0.90, bottom=0.08)
        
        axes_dict = {
            'plant_mag': fig.add_subplot(gs[0, 0]),
            'plant_phase': fig.add_subplot(gs[1, 0]),
            'ol_mag': fig.add_subplot(gs[0, 1]),
            'ol_phase': fig.add_subplot(gs[1, 1]),
            'cl_mag': fig.add_subplot(gs[0, 2]),
            'cl_phase': fig.add_subplot(gs[1, 2]),
            'dist_mag': fig.add_subplot(gs[0, 3]),
            'dist_phase': fig.add_subplot(gs[1, 3])
        }
        
    elif not include_plant and include_step:
        # 3 rows, 3 columns: OL, CL, Dist
        fig = plt.figure(figsize=(figsize[0]*0.8, figsize[1]))
        gs = fig.add_gridspec(3, 3, height_ratios=[1, 1, 1.2], 
                             hspace=0.35, wspace=0.25,
                             left=0.08, right=0.92, top=0.92, bottom=0.06)
        
        axes_dict = {
            'ol_mag': fig.add_subplot(gs[0, 0]),
            'ol_phase': fig.add_subplot(gs[1, 0]),
            'cl_mag': fig.add_subplot(gs[0, 1]),
            'cl_phase': fig.add_subplot(gs[1, 1]),
            'dist_mag': fig.add_subplot(gs[0, 2]),
            'dist_phase': fig.add_subplot(gs[1, 2]),
            'step': fig.add_subplot(gs[2, :])  # Step response spans all columns
        }
        
    else:
        # 2 rows, 3 columns: OL, CL, Dist
        fig = plt.figure(figsize=(figsize[0]*0.8, figsize[1]*0.6))
        gs = fig.add_gridspec(2, 3, hspace=0.35, wspace=0.25,
                             left=0.08, right=0.92, top=0.90, bottom=0.08)
        
        axes_dict = {
            'ol_mag': fig.add_subplot(gs[0, 0]),
            'ol_phase': fig.add_subplot(gs[1, 0]),
            'cl_mag': fig.add_subplot(gs[0, 1]),
            'cl_phase': fig.add_subplot(gs[1, 1]),
            'dist_mag': fig.add_subplot(gs[0, 2]),
            'dist_phase': fig.add_subplot(gs[1, 2])
        }
    
    # Add main title with better positioning
    fig.suptitle(title, fontsize=14, y=0.98, fontweight='bold', color='white')
    
    # Set up titles for each subplot with better formatting
    subplot_titles = {
        'plant_mag': 'Plant Model',
        'ol_mag': 'Open-Loop',
        'cl_mag': 'Closed-Loop (Tracking)',
        'dist_mag': 'Disturbance Rejection',
        'step': 'Step Response'
    }
    
    for key, title_text in subplot_titles.items():
        if key in axes_dict:
            axes_dict[key].set_title(title_text, color='white', fontsize=11, pad=8)
    
    # Set up x-axis labels only for bottom plots
    if 'plant_phase' in axes_dict:
        axes_dict['plant_phase'].set_xlabel('Frequency (Hz)', fontsize=10)
    
    axes_dict['ol_phase'].set_xlabel('Frequency (Hz)', fontsize=10)
    axes_dict['cl_phase'].set_xlabel('Frequency (Hz)', fontsize=10)
    axes_dict['dist_phase'].set_xlabel('Frequency (Hz)', fontsize=10)
    
    if 'step' in axes_dict:
        axes_dict['step'].set_xlabel('Time (s)', fontsize=10)
        axes_dict['step'].set_ylabel('Amplitude (p.u.)', fontsize=10)
    
    return fig, axes_dict


def plot_bode_subplot_pair(axes_mag, axes_phase, tf, label, frequency_range_hz=(0.1, 1000)):
    """
    Plot Bode magnitude and phase response for a transfer function on specific subplot axes.
    
    Args:
        axes_mag: Magnitude subplot axis
        axes_phase: Phase subplot axis
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
    axes_mag.semilogx(freq_hz, 20 * np.log10(mag), label=label)
    
    # Plot phase (degrees) with unwrapping to avoid discontinuities
    phase_unwrapped = np.unwrap(phase)  # Unwrap phase to avoid ±180° jumps
    axes_phase.semilogx(freq_hz, np.rad2deg(phase_unwrapped), label=label)


def plot_step_subplot(ax, tf, label, time_span_s=None, num_points=1000):
    """
    Plot step response for a transfer function on a specific subplot axis.
    
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


def finalize_comprehensive_analysis_figure(fig, axes_dict, frequency_range_hz=(0.1, 1000), 
                                          common_legend=True):
    """
    Apply standard formatting to the comprehensive analysis figure.
    
    Args:
        fig: Figure handle
        axes_dict: Dictionary of axes from setup_comprehensive_analysis_figure
        frequency_range_hz: Frequency range for x-axis limits
        common_legend: Whether to create a common legend instead of individual legends
    """
    # Format all Bode plot pairs first
    bode_pairs = [
        ('plant_mag', 'plant_phase'),
        ('ol_mag', 'ol_phase'), 
        ('cl_mag', 'cl_phase'),
        ('dist_mag', 'dist_phase')
    ]
    
    for mag_key, phase_key in bode_pairs:
        if mag_key in axes_dict and phase_key in axes_dict:
            # Magnitude subplot formatting
            axes_dict[mag_key].set_ylabel('Magnitude (dB)', fontsize=10)
            axes_dict[mag_key].grid(True, which='both', alpha=0.3)
            axes_dict[mag_key].set_xlim(frequency_range_hz)
            axes_dict[mag_key].tick_params(axis='both', which='major', labelsize=9)
            # Add 0 dB reference line
            axes_dict[mag_key].axhline(y=0, color='red', linestyle='--', alpha=0.7, linewidth=1.5)
            
            # Phase subplot formatting  
            axes_dict[phase_key].set_ylabel('Phase (deg)', fontsize=10)
            axes_dict[phase_key].grid(True, which='both', alpha=0.3)
            axes_dict[phase_key].set_xlim(frequency_range_hz)
            axes_dict[phase_key].tick_params(axis='both', which='major', labelsize=9)
            # Add -180° reference line
            axes_dict[phase_key].axhline(y=-180, color='red', linestyle='--', alpha=0.7, linewidth=1.5)
    
    # Format step response if present
    if 'step' in axes_dict:
        axes_dict['step'].grid(True, alpha=0.3)
        axes_dict['step'].tick_params(axis='both', which='major', labelsize=9)
    
    # Collect ALL unique labels and handles from ALL subplots
    if common_legend:
        all_handles = []
        all_labels = []
        seen_labels = set()
        
        # Collect from all axes - only process each unique label once
        for ax_key, ax in axes_dict.items():
            handles, labels = ax.get_legend_handles_labels()
            for handle, label in zip(handles, labels):
                if label not in seen_labels and label and not label.startswith('_'):
                    all_handles.append(handle)
                    all_labels.append(label)
                    seen_labels.add(label)
        
        # Remove any existing individual legends
        for ax in axes_dict.values():
            legend = ax.get_legend()
            if legend is not None:
                legend.remove()
        
        # Create common legend inside the step response plot if we have data
        if all_handles:
            # Use normal tight layout since legend will be inside step plot
            plt.tight_layout(rect=[0.02, 0.02, 0.98, 0.94])
            
            # Place legend inside the step response subplot (lower right)
            if 'step' in axes_dict:
                legend = axes_dict['step'].legend(all_handles, all_labels,
                                                loc='lower right',
                                                ncol=1,
                                                frameon=True,
                                                facecolor='#3E3E3E',
                                                edgecolor='#6E6E6E',
                                                fontsize='x-small',
                                                title='Scenarios',
                                                title_fontsize='small',
                                                borderaxespad=0.5,
                                                columnspacing=1.0,
                                                handlelength=2.0,
                                                handletextpad=0.8,
                                                framealpha=0.9)  # Slight transparency
            else:
                # Fallback: use figure legend if no step subplot
                legend = fig.legend(all_handles, all_labels,
                                  bbox_to_anchor=(0.98, 0.02),
                                  loc='lower right',
                                  ncol=1,
                                  frameon=True,
                                  facecolor='#3E3E3E',
                                  edgecolor='#6E6E6E',
                                  fontsize='x-small',
                                  title='Scenarios',
                                  title_fontsize='small',
                                  borderaxespad=0.5,
                                  columnspacing=1.0,
                                  handlelength=2.0,
                                  handletextpad=0.8,
                                  framealpha=0.9)
            
            # Store line references for all axes to enable toggling
            lines_by_label = {}
            for ax in axes_dict.values():
                for line in ax.get_lines():
                    label = line.get_label()
                    if label and not label.startswith('_'):  # Skip internal matplotlib labels
                        if label not in lines_by_label:
                            lines_by_label[label] = []
                        lines_by_label[label].append(line)
            
            # Make legend interactive - clicking toggles line visibility in ALL subplots
            toggled_off = set()  # Track which labels are toggled off
            
            def on_pick(event):
                if event.artist in legend.get_lines():
                    # Find which legend line was clicked
                    legend_line = event.artist
                    idx = legend.get_lines().index(legend_line)
                    
                    if idx < len(all_labels):
                        label = all_labels[idx]
                        
                        # Toggle all lines with this label in all subplots
                        if label in lines_by_label:
                            # Toggle visibility
                            if label in toggled_off:
                                toggled_off.remove(label)
                                new_visible = True
                            else:
                                toggled_off.add(label)
                                new_visible = False
                            
                            # Apply to all lines with this label
                            for line in lines_by_label[label]:
                                line.set_visible(new_visible)
                            
                            # Update legend appearance
                            legend_line.set_alpha(0.3 if not new_visible else 1.0)
                            legend.get_texts()[idx].set_alpha(0.3 if not new_visible else 1.0)
                        
                        fig.canvas.draw_idle()
            
            # Make legend lines pickable
            for line in legend.get_lines():
                line.set_picker(5)  # 5 points tolerance
            
            # Connect the pick event
            fig.canvas.mpl_connect('pick_event', on_pick)
            
            # Add instruction text at bottom of figure
            fig.text(0.02, 0.01, 'Click legend items to toggle visibility', 
                    fontsize=7, style='italic', color='#AAAAAA',
                    ha='left', va='bottom')
    else:
        # If not using common legend, add individual legends
        for mag_key, phase_key in bode_pairs:
            if mag_key in axes_dict and phase_key in axes_dict:
                axes_dict[mag_key].legend(fontsize='x-small', loc='best')
                axes_dict[phase_key].legend(fontsize='x-small', loc='best')
        
        if 'step' in axes_dict:
            axes_dict['step'].legend(fontsize='x-small', loc='best')
        
        plt.tight_layout(rect=[0.02, 0.02, 0.98, 0.94])


def configure_plot_style():
    """Configure matplotlib style for consistent dark theme appearance."""
    try:
        plt.style.use('dark_background')
    except:
        # Fallback to manual dark theme configuration
        plt.style.use('default')
    
    # Dark theme configuration
    plt.rcParams['figure.facecolor'] = '#2E2E2E'
    plt.rcParams['axes.facecolor'] = '#3E3E3E'
    plt.rcParams['axes.edgecolor'] = '#CCCCCC'
    plt.rcParams['axes.labelcolor'] = '#FFFFFF'
    plt.rcParams['xtick.color'] = '#CCCCCC'
    plt.rcParams['ytick.color'] = '#CCCCCC'
    plt.rcParams['text.color'] = '#FFFFFF'
    plt.rcParams['grid.color'] = '#555555'
    plt.rcParams['grid.alpha'] = 0.3
    plt.rcParams['axes.grid'] = True
    plt.rcParams['legend.facecolor'] = '#3E3E3E'
    plt.rcParams['legend.edgecolor'] = '#CCCCCC'
