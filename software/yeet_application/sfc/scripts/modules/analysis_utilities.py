"""
Analysis Utilities for LCI Control Systems

This module provides common plotting functions and analysis tools for
current and speed control loop analysis.
"""

import numpy as np
import matplotlib.pyplot as plt
import control as ct
from matplotlib.widgets import Cursor


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
    
    # Add interactive cursors
    cursors = []
    for i, ax in enumerate(axes):
        cursor = Cursor(ax, useblit=True, color='yellow', linewidth=1, alpha=0.8)
        cursors.append(cursor)
        
        # Add value display annotation
        text_annotation = ax.annotate('', xy=(0, 0), xytext=(20, 20),
                                    textcoords="offset points",
                                    bbox=dict(boxstyle="round,pad=0.3",
                                            facecolor='black',
                                            edgecolor='yellow',
                                            alpha=0.8),
                                    fontsize=8, color='white',
                                    visible=False)
        ax._cursor_annotation = text_annotation
        
        # Connect mouse motion event
        def on_mouse_move(event, ax=ax, is_mag=(i==0)):
            if event.inaxes == ax and ax.get_lines():
                x_cursor = event.xdata
                y_cursor = event.ydata
                
                if x_cursor is not None and y_cursor is not None:
                    if is_mag:
                        text = f'f: {x_cursor:.2f} Hz\nMag: {y_cursor:.2f} dB'
                    else:
                        text = f'f: {x_cursor:.2f} Hz\nPhase: {y_cursor:.1f}°'
                    
                    ax._cursor_annotation.set_text(text)
                    ax._cursor_annotation.xy = (x_cursor, y_cursor)
                    ax._cursor_annotation.set_visible(True)
                    fig.canvas.draw_idle()
                else:
                    ax._cursor_annotation.set_visible(False)
                    fig.canvas.draw_idle()
            else:
                if hasattr(ax, '_cursor_annotation'):
                    ax._cursor_annotation.set_visible(False)
                    fig.canvas.draw_idle()
        
        fig.canvas.mpl_connect('motion_notify_event', 
                             lambda event, ax=ax, is_mag=(i==0): on_mouse_move(event, ax, is_mag))
    
    return cursors


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
            axes_dict[mag_key].axhline(y=0, color='white', linestyle='--', alpha=0.7, linewidth=1.5)
            
            # Phase subplot formatting  
            axes_dict[phase_key].set_ylabel('Phase (deg)', fontsize=10)
            axes_dict[phase_key].grid(True, which='both', alpha=0.3)
            axes_dict[phase_key].set_xlim(frequency_range_hz)
            axes_dict[phase_key].tick_params(axis='both', which='major', labelsize=9)
            # Add -180° reference line
            axes_dict[phase_key].axhline(y=-180, color='white', linestyle='--', alpha=0.7, linewidth=1.5)
    
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
            fig.text(0.02, 0.01, 'Click legend items to toggle visibility | Left-click plots to mark points | Right-click plots to clear markers', 
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
    
    # Add interactive cursors to all plots
    add_interactive_cursors(fig, axes_dict)


def add_interactive_cursors(fig, axes_dict):
    """
    Add interactive crosshair cursors with axis lines to all Bode plot axes.
    
    Args:
        fig: Figure handle
        axes_dict: Dictionary of axes from setup_comprehensive_analysis_figure
    """
    cursors = []
    
    # Add cursors to all Bode plot axes (magnitude and phase)
    bode_axes_keys = [
        'plant_mag', 'plant_phase',
        'ol_mag', 'ol_phase',
        'cl_mag', 'cl_phase', 
        'dist_mag', 'dist_phase'
    ]
    
    for ax_key in bode_axes_keys:
        if ax_key in axes_dict:
            ax = axes_dict[ax_key]
            
            # Create interactive axis lines for crosshair effect
            # Vertical line (frequency)
            vline = ax.axvline(x=0, color='white', linestyle='--', linewidth=0.7, alpha=0.7, visible=False)
            # Horizontal line (magnitude/phase)
            hline = ax.axhline(y=0, color='white', linestyle='--', linewidth=0.7, alpha=0.7, visible=False)
            
            # Store references to the lines
            ax._cursor_vline = vline
            ax._cursor_hline = hline
            
            # Add text annotation for displaying values
            text_annotation = ax.annotate('', xy=(0, 0), xytext=(20, 20), 
                                        textcoords="offset points",
                                        bbox=dict(boxstyle="round,pad=0.3", 
                                                facecolor='black', 
                                                edgecolor='grey',
                                                alpha=0.8),
                                        fontsize=8, color='white',
                                        visible=False)
            
            # Store reference to annotation in the axis for later use
            ax._cursor_annotation = text_annotation
            
            # Initialize persistent markers list
            ax._persistent_markers = []
            ax._persistent_annotations = []
            
            # Determine if this is a magnitude plot (fix closure issue)
            is_magnitude_plot = 'mag' in ax_key
            
            # Connect mouse motion event for value display
            def on_mouse_move(event, ax=ax, is_mag=is_magnitude_plot, ax_key_local=ax_key):
                if event.inaxes == ax:
                    # Get cursor position
                    x_cursor = event.xdata
                    y_cursor = event.ydata
                    
                    if x_cursor is not None and y_cursor is not None:
                        # Find the closest point on any of the plotted lines
                        closest_x = x_cursor
                        closest_y = y_cursor
                        min_distance = float('inf')
                        closest_line = None
                        
                        # Check all lines in this axis
                        for line in ax.get_lines():
                            if line.get_visible() and not line.get_label().startswith('_'):
                                xdata = line.get_xdata()
                                ydata = line.get_ydata()
                                
                                if len(xdata) > 0 and len(ydata) > 0:
                                    # Find closest x point in log scale
                                    log_x_cursor = np.log10(x_cursor) if x_cursor > 0 else -10
                                    log_xdata = np.log10(np.maximum(xdata, 1e-10))  # Avoid log(0)
                                    
                                    # Find the closest frequency point
                                    idx = np.argmin(np.abs(log_xdata - log_x_cursor))
                                    
                                    # Get the corresponding y value
                                    snap_x = xdata[idx]
                                    snap_y = ydata[idx]
                                    
                                    # Calculate distance in plot coordinates (considering log scale for x)
                                    if snap_x > 0:
                                        x_dist = abs(np.log10(snap_x) - log_x_cursor)
                                    else:
                                        x_dist = float('inf')
                                    
                                    # Normalize y distance based on plot range
                                    y_range = ax.get_ylim()[1] - ax.get_ylim()[0]
                                    y_dist = abs(snap_y - y_cursor) / y_range if y_range > 0 else 0
                                    
                                    # Combined distance metric (weighted more towards x)
                                    distance = x_dist * 2 + y_dist
                                    
                                    if distance < min_distance:
                                        min_distance = distance
                                        closest_x = snap_x
                                        closest_y = snap_y
                                        closest_line = line
                        
                        # Update crosshair lines to snap to closest point
                        ax._cursor_vline.set_xdata([closest_x, closest_x])
                        ax._cursor_hline.set_ydata([closest_y, closest_y])
                        ax._cursor_vline.set_visible(True)
                        ax._cursor_hline.set_visible(True)
                        
                        # Format the display text to show both magnitude and phase
                        # Find the paired axis to get both mag and phase values
                        mag_value = None
                        phase_value = None
                        
                        # Determine the axis pair key
                        axis_base = ax_key_local.replace('_mag', '').replace('_phase', '')
                        mag_ax_key = f"{axis_base}_mag"
                        phase_ax_key = f"{axis_base}_phase"
                        
                        # Get values from both magnitude and phase axes at the same frequency
                        if mag_ax_key in axes_dict and phase_ax_key in axes_dict:
                            # Get magnitude value
                            mag_ax = axes_dict[mag_ax_key]
                            for line in mag_ax.get_lines():
                                if (line.get_visible() and not line.get_label().startswith('_') and 
                                    closest_line and line.get_label() == closest_line.get_label()):
                                    xdata = line.get_xdata()
                                    ydata = line.get_ydata()
                                    if len(xdata) > 0:
                                        log_x_target = np.log10(closest_x) if closest_x > 0 else -10
                                        log_xdata = np.log10(np.maximum(xdata, 1e-10))
                                        idx = np.argmin(np.abs(log_xdata - log_x_target))
                                        mag_value = ydata[idx]
                                        break
                            
                            # Get phase value  
                            phase_ax = axes_dict[phase_ax_key]
                            for line in phase_ax.get_lines():
                                if (line.get_visible() and not line.get_label().startswith('_') and 
                                    closest_line and line.get_label() == closest_line.get_label()):
                                    xdata = line.get_xdata()
                                    ydata = line.get_ydata()
                                    if len(xdata) > 0:
                                        log_x_target = np.log10(closest_x) if closest_x > 0 else -10
                                        log_xdata = np.log10(np.maximum(xdata, 1e-10))
                                        idx = np.argmin(np.abs(log_xdata - log_x_target))
                                        phase_value = ydata[idx]
                                        break
                        
                        # Build text with both values when available
                        text = f'f: {closest_x:.2f} Hz'
                        if mag_value is not None and phase_value is not None:
                            text += f'\nMag: {mag_value:.2f} dB\nPhase: {phase_value:.1f}°'
                        elif is_mag:
                            text += f'\nMag: {closest_y:.2f} dB'
                        else:
                            text += f'\nPhase: {closest_y:.1f}°'
                        
                        if closest_line:
                            text += f'\nCurve: {closest_line.get_label()}'
                        
                        # Update annotation to show at the snapped position
                        ax._cursor_annotation.set_text(text)
                        ax._cursor_annotation.xy = (closest_x, closest_y)
                        ax._cursor_annotation.set_visible(True)
                        fig.canvas.draw_idle()
                    else:
                        # Hide crosshair lines and annotation
                        ax._cursor_vline.set_visible(False)
                        ax._cursor_hline.set_visible(False)
                        ax._cursor_annotation.set_visible(False)
                        fig.canvas.draw_idle()
                else:
                    # Mouse is outside this axis
                    if hasattr(ax, '_cursor_annotation'):
                        ax._cursor_vline.set_visible(False)
                        ax._cursor_hline.set_visible(False)
                        ax._cursor_annotation.set_visible(False)
                        fig.canvas.draw_idle()
            
            fig.canvas.mpl_connect('motion_notify_event', 
                                 lambda event, ax=ax, is_mag=is_magnitude_plot, ax_key_local=ax_key: on_mouse_move(event, ax, is_mag, ax_key_local))
            
            # Add click handler for persistent markers
            def on_click(event, ax=ax, is_mag=is_magnitude_plot, ax_key_local=ax_key):
                if event.inaxes == ax and event.button == 1:  # Left click
                    x_cursor = event.xdata
                    y_cursor = event.ydata
                    
                    if x_cursor is not None and y_cursor is not None:
                        # Find the closest point (same logic as mouse move)
                        closest_x = x_cursor
                        closest_y = y_cursor
                        min_distance = float('inf')
                        closest_line = None
                        
                        for line in ax.get_lines():
                            if line.get_visible() and not line.get_label().startswith('_'):
                                xdata = line.get_xdata()
                                ydata = line.get_ydata()
                                
                                if len(xdata) > 0 and len(ydata) > 0:
                                    log_x_cursor = np.log10(x_cursor) if x_cursor > 0 else -10
                                    log_xdata = np.log10(np.maximum(xdata, 1e-10))
                                    idx = np.argmin(np.abs(log_xdata - log_x_cursor))
                                    
                                    snap_x = xdata[idx]
                                    snap_y = ydata[idx]
                                    
                                    if snap_x > 0:
                                        x_dist = abs(np.log10(snap_x) - log_x_cursor)
                                    else:
                                        x_dist = float('inf')
                                    
                                    y_range = ax.get_ylim()[1] - ax.get_ylim()[0]
                                    y_dist = abs(snap_y - y_cursor) / y_range if y_range > 0 else 0
                                    distance = x_dist * 2 + y_dist
                                    
                                    if distance < min_distance:
                                        min_distance = distance
                                        closest_x = snap_x
                                        closest_y = snap_y
                                        closest_line = line
                        
                        # Create persistent marker
                        marker_color = closest_line.get_color() if closest_line else 'red'
                        marker = ax.plot(closest_x, closest_y, marker='o', markersize=8, 
                                       markerfacecolor=marker_color, markeredgecolor='white', 
                                       markeredgewidth=2, linestyle='', zorder=10)[0]
                        
                        # Create persistent annotation with both mag and phase values
                        # Find the paired axis to get both mag and phase values
                        mag_value = None
                        phase_value = None
                        
                        # Determine the axis pair key
                        axis_base = ax_key_local.replace('_mag', '').replace('_phase', '')
                        mag_ax_key = f"{axis_base}_mag"
                        phase_ax_key = f"{axis_base}_phase"
                        
                        # Get values from both magnitude and phase axes at the same frequency
                        if mag_ax_key in axes_dict and phase_ax_key in axes_dict:
                            # Get magnitude value
                            mag_ax = axes_dict[mag_ax_key]
                            for line in mag_ax.get_lines():
                                if (line.get_visible() and not line.get_label().startswith('_') and 
                                    closest_line and line.get_label() == closest_line.get_label()):
                                    xdata = line.get_xdata()
                                    ydata = line.get_ydata()
                                    if len(xdata) > 0:
                                        log_x_target = np.log10(closest_x) if closest_x > 0 else -10
                                        log_xdata = np.log10(np.maximum(xdata, 1e-10))
                                        idx = np.argmin(np.abs(log_xdata - log_x_target))
                                        mag_value = ydata[idx]
                                        break
                            
                            # Get phase value  
                            phase_ax = axes_dict[phase_ax_key]
                            for line in phase_ax.get_lines():
                                if (line.get_visible() and not line.get_label().startswith('_') and 
                                    closest_line and line.get_label() == closest_line.get_label()):
                                    xdata = line.get_xdata()
                                    ydata = line.get_ydata()
                                    if len(xdata) > 0:
                                        log_x_target = np.log10(closest_x) if closest_x > 0 else -10
                                        log_xdata = np.log10(np.maximum(xdata, 1e-10))
                                        idx = np.argmin(np.abs(log_xdata - log_x_target))
                                        phase_value = ydata[idx]
                                        break
                        
                        # Build text with both values when available
                        text = f'f: {closest_x:.2f} Hz'
                        if mag_value is not None and phase_value is not None:
                            text += f'\nMag: {mag_value:.2f} dB\nPhase: {phase_value:.1f}°'
                        elif is_mag:
                            text += f'\nMag: {closest_y:.2f} dB'
                        else:
                            text += f'\nPhase: {closest_y:.1f}°'
                        
                        if closest_line:
                            text += f'\n{closest_line.get_label()}'
                        
                        persistent_annotation = ax.annotate(text, xy=(closest_x, closest_y), 
                                                          xytext=(10, 10), textcoords="offset points",
                                                          bbox=dict(boxstyle="round,pad=0.3",
                                                                  facecolor=marker_color,
                                                                  edgecolor='white',
                                                                  alpha=0.8),
                                                          fontsize=7, color='white',
                                                          arrowprops=dict(arrowstyle='->', 
                                                                        connectionstyle='arc3,rad=0',
                                                                        color='white', lw=1))
                        
                        # Store references
                        ax._persistent_markers.append(marker)
                        ax._persistent_annotations.append(persistent_annotation)
                        
                        fig.canvas.draw_idle()
                
                elif event.inaxes == ax and event.button == 3:  # Right click to clear markers
                    # Remove all persistent markers and annotations
                    for marker in ax._persistent_markers:
                        marker.remove()
                    for annotation in ax._persistent_annotations:
                        annotation.remove()
                    ax._persistent_markers.clear()
                    ax._persistent_annotations.clear()
                    fig.canvas.draw_idle()
            
            fig.canvas.mpl_connect('button_press_event',
                                 lambda event, ax=ax, is_mag=is_magnitude_plot, ax_key_local=ax_key: on_click(event, ax, is_mag, ax_key_local))
    
    # Add cursor to step response if present
    if 'step' in axes_dict:
        step_ax = axes_dict['step']
        
        # Create interactive axis lines for step response
        step_vline = step_ax.axvline(x=0, color='yellow', linestyle='-', linewidth=1, alpha=0.7, visible=False)
        step_hline = step_ax.axhline(y=0, color='yellow', linestyle='-', linewidth=1, alpha=0.7, visible=False)
        
        step_ax._cursor_vline = step_vline
        step_ax._cursor_hline = step_hline
        
        # Add text annotation for step response
        step_text = step_ax.annotate('', xy=(0, 0), xytext=(20, 20),
                                   textcoords="offset points",
                                   bbox=dict(boxstyle="round,pad=0.3",
                                           facecolor='black',
                                           edgecolor='yellow',
                                           alpha=0.8),
                                   fontsize=8, color='white',
                                   visible=False)
        step_ax._cursor_annotation = step_text
        
        # Initialize persistent markers for step response
        step_ax._persistent_markers = []
        step_ax._persistent_annotations = []
        
        def on_step_mouse_move(event):
            if event.inaxes == step_ax:
                x_cursor = event.xdata
                y_cursor = event.ydata
                
                if x_cursor is not None and y_cursor is not None:
                    # Find the closest point on any of the plotted lines
                    closest_x = x_cursor
                    closest_y = y_cursor
                    min_distance = float('inf')
                    closest_line = None
                    
                    # Check all lines in the step response axis
                    for line in step_ax.get_lines():
                        if line.get_visible() and not line.get_label().startswith('_'):
                            xdata = line.get_xdata()
                            ydata = line.get_ydata()
                            
                            if len(xdata) > 0 and len(ydata) > 0:
                                # Find the closest time point
                                idx = np.argmin(np.abs(xdata - x_cursor))
                                
                                # Get the corresponding y value
                                snap_x = xdata[idx]
                                snap_y = ydata[idx]
                                
                                # Calculate distance in plot coordinates
                                x_range = step_ax.get_xlim()[1] - step_ax.get_xlim()[0]
                                y_range = step_ax.get_ylim()[1] - step_ax.get_ylim()[0]
                                
                                x_dist = abs(snap_x - x_cursor) / x_range if x_range > 0 else 0
                                y_dist = abs(snap_y - y_cursor) / y_range if y_range > 0 else 0
                                
                                # Combined distance metric
                                distance = x_dist * 2 + y_dist  # Weight x more heavily
                                
                                if distance < min_distance:
                                    min_distance = distance
                                    closest_x = snap_x
                                    closest_y = snap_y
                                    closest_line = line
                    
                    # Update crosshair lines to snap to closest point
                    step_ax._cursor_vline.set_xdata([closest_x, closest_x])
                    step_ax._cursor_hline.set_ydata([closest_y, closest_y])
                    step_ax._cursor_vline.set_visible(True)
                    step_ax._cursor_hline.set_visible(True)
                    
                    text = f't: {closest_x:.3f} s\nAmpl: {closest_y:.3f} p.u.'
                    if closest_line:
                        text += f'\nCurve: {closest_line.get_label()}'
                        
                    step_ax._cursor_annotation.set_text(text)
                    step_ax._cursor_annotation.xy = (closest_x, closest_y)
                    step_ax._cursor_annotation.set_visible(True)
                    fig.canvas.draw_idle()
                else:
                    step_ax._cursor_vline.set_visible(False)
                    step_ax._cursor_hline.set_visible(False)
                    step_ax._cursor_annotation.set_visible(False)
                    fig.canvas.draw_idle()
            else:
                if hasattr(step_ax, '_cursor_annotation'):
                    step_ax._cursor_vline.set_visible(False)
                    step_ax._cursor_hline.set_visible(False)
                    step_ax._cursor_annotation.set_visible(False)
                    fig.canvas.draw_idle()
        
        fig.canvas.mpl_connect('motion_notify_event', on_step_mouse_move)
        
        # Add click handler for persistent markers on step response
        def on_step_click(event):
            if event.inaxes == step_ax and event.button == 1:  # Left click
                x_cursor = event.xdata
                y_cursor = event.ydata
                
                if x_cursor is not None and y_cursor is not None:
                    # Find the closest point (same logic as mouse move)
                    closest_x = x_cursor
                    closest_y = y_cursor
                    min_distance = float('inf')
                    closest_line = None
                    
                    for line in step_ax.get_lines():
                        if line.get_visible() and not line.get_label().startswith('_'):
                            xdata = line.get_xdata()
                            ydata = line.get_ydata()
                            
                            if len(xdata) > 0 and len(ydata) > 0:
                                idx = np.argmin(np.abs(xdata - x_cursor))
                                snap_x = xdata[idx]
                                snap_y = ydata[idx]
                                
                                x_range = step_ax.get_xlim()[1] - step_ax.get_xlim()[0]
                                y_range = step_ax.get_ylim()[1] - step_ax.get_ylim()[0]
                                x_dist = abs(snap_x - x_cursor) / x_range if x_range > 0 else 0
                                y_dist = abs(snap_y - y_cursor) / y_range if y_range > 0 else 0
                                distance = x_dist * 2 + y_dist
                                
                                if distance < min_distance:
                                    min_distance = distance
                                    closest_x = snap_x
                                    closest_y = snap_y
                                    closest_line = line
                    
                    # Create persistent marker
                    marker_color = closest_line.get_color() if closest_line else 'red'
                    marker = step_ax.plot(closest_x, closest_y, marker='o', markersize=8,
                                        markerfacecolor=marker_color, markeredgecolor='white',
                                        markeredgewidth=2, linestyle='', zorder=10)[0]
                    
                    # Create persistent annotation
                    text = f't: {closest_x:.3f} s\nAmpl: {closest_y:.3f} p.u.'
                    if closest_line:
                        text += f'\n{closest_line.get_label()}'
                    
                    persistent_annotation = step_ax.annotate(text, xy=(closest_x, closest_y),
                                                           xytext=(10, 10), textcoords="offset points",
                                                           bbox=dict(boxstyle="round,pad=0.3",
                                                                   facecolor=marker_color,
                                                                   edgecolor='white',
                                                                   alpha=0.8),
                                                           fontsize=7, color='white',
                                                           arrowprops=dict(arrowstyle='->',
                                                                         connectionstyle='arc3,rad=0',
                                                                         color='white', lw=1))
                    
                    # Store references
                    step_ax._persistent_markers.append(marker)
                    step_ax._persistent_annotations.append(persistent_annotation)
                    
                    fig.canvas.draw_idle()
            
            elif event.inaxes == step_ax and event.button == 3:  # Right click to clear
                # Remove all persistent markers and annotations
                for marker in step_ax._persistent_markers:
                    marker.remove()
                for annotation in step_ax._persistent_annotations:
                    annotation.remove()
                step_ax._persistent_markers.clear()
                step_ax._persistent_annotations.clear()
                fig.canvas.draw_idle()
        
        fig.canvas.mpl_connect('button_press_event', on_step_click)
    
    return cursors


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
