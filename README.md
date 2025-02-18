# Sensor Noise Filter
This is a ROS package for sensor (gaussian) noise filtering. This package consists of different filters (e.g. moving average, exponential moving average, butterworth) and signal visualization tool.

## Content
1. [Moving Average (MA)](#moving-average-ma)
2. [Exponential Moving Average (EMA)](#exponential-moving-average-ema)
3. [Butterworth Filter](#butterworth-filter)
4. [Signal Visualization](#signal-visualization)
5. [Installation](#installation)

## Moving Average (MA)
`ma_filter.py` subscribes to a sensor topic e.g. `sensor/dvl` and publishes the filtered data to the output topic e.g. `sensor/dvl_filtered`
### Theory
The current filtered value is taken as the average of n past values (including the current value), where n is the window size, the weightage of n past datas in current filtered data is same (average). Larger window size reduces more gaussian noise but results in more lag.
### Parameter
- `input_sensor_topic` (default `"/sensor/dvl"`)
- `output_sensor_topic` (default `"/sensor/dvl_filtered"`)
- `window_size` (default `5`)
### Usage
rosrun (optional param specification at the end)
```
rosrun sensor_noise_filter ma_filter.py _window_size:=8
```
roslaunch with param specification, add this node to your launch file
```
<node name="ma_filter" pkg="sensor_noise_filter" type="ma_filter.py" output="screen">
  <param name="input_sensor_topic" value="/sensor/dvl" />
  <param name="output_sensor_topic" value="/sensor/dvl_filtered" />
  <param name="window_size" value="5" />
</node>
```

## Exponential Moving Average (EMA)
`ema_filter.py` subscribes to a sensor topic e.g. `sensor/dvl` and publishes the filtered data to the output topic e.g. `sensor/dvl_filtered`
### Theory
The current filtered value is taken as alpha*current_value + (1 - alpha)*last_filtered_value. Recent data takes more weightage (depends on alpha). alpha=1 means no filtering, only current data is considered, high alpha give more weightage to current and recent data. Low alpha reduces more gaussian noise but results in more lag.
### Parameter
- `input_sensor_topic` (default `"/sensor/dvl"`)
- `output_sensor_topic` (default `"/sensor/dvl_filtered"`)
- `alpha` (default `0.5`)
### Usage
rosrun (optional param specification at the end)
```
rosrun sensor_noise_filter ema_filter.py _alpha:=0.7
```
roslaunch with param specification, add this node to your launch file
```
<node name="ema_filter" pkg="sensor_noise_filter" type="ema_filter.py" output="screen">
  <param name="input_sensor_topic" value="/sensor/dvl" />
  <param name="output_sensor_topic" value="/sensor/dvl_filtered" />
  <param name="alpha" value="0.5" />
</node>
```

## Butterworth Filter
`butterworth.py` subscribes to a sensor topic e.g. `sensor/dvl` and publishes the filtered data to the output topic e.g. `sensor/dvl_filtered`
### Theory
Butterworth filter is a low pass filter, eliminating high frequency gaussian noise. Low cutoff reduces more noise but results in higher delay. High order results in steeper roll-off (more restrictive to frequency beyond cutoff), but more delay and computational cost. More theoretical details can refer to https://www.youtube.com/watch?v=dmzikG1jZpU&list=PLdciPPorsHunK-nh9PyXcA8rTy4m-TuOV.
### Parameter
- `input_sensor_topic` (default `"/sensor/dvl"`)
- `output_sensor_topic` (default `"/sensor/dvl_filtered"`)
- `cutoff` (default `10`)
- `fs` (default `33`)
- `order` (default `4`)
### Usage
rosrun (optional param specification at the end)
```
rosrun sensor_noise_filter butterworth.py _order:=8 _fs:=50
```
roslaunch with param specification, add this node to your launch file
```
<node name="butterworth" pkg="sensor_noise_filter" type="butterworth.py" output="screen">
  <param name="input_sensor_topic" value="/sensor/dvl" />
  <param name="output_sensor_topic" value="/sensor/dvl_filtered" />
  <param name="cutoff" value="5" />
  <param name="fs" value="33" />
  <param name="order" value="4" />
</node>
```
> NOTE: Run [plot_frequency.py](#plotfrequencypy) to determine an appropriate cutoff frequency.

## Signal Visualization

### `plot_frequency.py`
Subscribes to a sensor topic to plot its frequency domain signal. The plot is shown on the screen after the script stopped.
### Parameter
- `input_sensor_topic` (default `"/sensor/dvl"`)
- `fs` (default `33`)
### Usage
rosrun (optional param specification at the end), press `Ctrl+C` after some time (for data collection) to end the script and show frequency plot.
```
rosrun sensor_noise_filter plot_frequency.py _input_sensor_topic:="/imu" _fs:=50
```

### `plot_dvl.py`
Subscribes to clean, noisy, & multiple filtered signal topic to evaluate and visualize filters' performance. It dynamically subscribes to filtered topics with common prefix e.g. `sensor/dvl_filter` (check available filtered topics every 0.3s). The performance metrics (snr, lag) is printed on console & the plot is saved in `image_plots/` after the script stopped. The lag of filtered signal is calculated and compensated before SNR calculation using cross correlation. This is to eliminate the effect of lag on SNR. SNR calculated in this way may suffer from inconsistent sampling rate. Resolution of lag calculation depends on sampling rate.
### Parameter
- `clean_sensor_topic` (default `"/sensor/dvl_clean"`)
- `noisy_sensor_topic` (default `"/sensor/dvl"`)
- `prefix` (default `"/sensor/dvl_filtered"`): Prefix for filtered topics
- `interval` (default `0.3`): Interval to check available filtered topics
- `fs` (default `33`): sampling rate of all topics
- `plot_aligned_filtered_signal` (default `False`): Plot the aligned filtered signal with compensated lag on the same graph (mainly for debug use)
- `plots_dir` (default `image_plots`): The save directory of plots, relative to the package path e.g. `/path/to/sensor_noise_filter/image_plots`
### Usage
rosrun (optional param specification at the end), press `Ctrl+C` after some time (for data collection) to end the script, show metrics & save plots.
```
rosrun sensor_noise_filter plot_dvl.py _prefix:="/sensor/imu/filtered"
```
roslaunch: use [`plot.launch`](#plotlaunch)

### `plot.launch`
Run `plot_dvl.py` & filter script(s) at the same time to evaluate, visualize and compare the performance of different filters in the same graph. Inspect `plot.launch` to include desired filters for comparison.
### Usage
Run command below and press `Ctrl+C` after some time (for data collection) to show metrics, save plots & shutdown ROS. 
```
roslaunch sensor_noise_filter plot.launch
```

> NOTE: SNR & lag calculated by `plot_dvl.py` are not absolutely accurate due to inconsistent sensor sampling rate/missing datapoints. Run `plot_dvl.py` or `plot.launch` before publishing sensor data to ensure same data length/minimal data losses to increase snr & lag accuracy.

## Installation
```
cd /path/to/your/ROS_workspace/src
git clone https://github.com/bk1021/sensor_noise_filter.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```
