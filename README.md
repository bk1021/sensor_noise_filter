### Butterworth Guide

Run `butterworth.py` to publish filtered sensor data to e.g. `sensor/dvl_filtered`

To run with default param
```
rosrun sensor_noise_filter butterworth.py
```
To run with specified param
```
rosrun sensor_noise_filter butterworth.py _cutoff:=5 _fs:=33 _order:=4
```
To include this script in launch file with specified param
```
<node name="butterworth_1" pkg="sensor_noise_filter" type="butterworth.py" output="screen">
  <param name="input_sensor_topic" value="/sensor/dvl" />
  <param name="output_sensor_topic" value="/sensor/dvl_filtered_3" />
  <param name="cutoff" value="5" />
  <param name="fs" value="33" />
  <param name="order" value="4" />
  <param name="label" value="butter, cutoff=5,order=4" />
</node>
```

### Notes
1. Run `plot_frequency` and press `Ctrl+C` after some time & AUV motion to plot frequency domain sensor signal
2. Run `plot.launch` to evaluate, visualize and compare different filters' performance. Press `Ctrl+C` after some time & AUV motion to print metrics & save plots to `image_plots`
3. Inspect `plot.launch` to include desired filter for testing
4. SNR & Lag printed on console by `plot.launch` may not be very accurate due to inconsistent sampling rate/missing data, but it is still reliable.
5. Run `plot.launch` before playing Unity Scene to prevent data loss and increase metrics accuracy. 
