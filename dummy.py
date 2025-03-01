import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read the CSV file without headers and specify column names
data = pd.read_csv('test_setpoint_data.csv', header=None, names=['timestamp', 'x', 'y', 'angle'])

# Create a plot with a larger figure size and two subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 8), gridspec_kw={'width_ratios': [3, 1]})

# Plot each arrow and annotate with timestamp on the first subplot
for index, row in data.iterrows():
    x = row['x']
    y = row['y']
    angle = row['angle']
    timestamp = row['timestamp']
    
    # Calculate the arrow direction with shorter length
    dx = 0.5 * np.cos(angle)
    dy = 0.5 * np.sin(angle)
    
    # Plot the arrow
    ax1.arrow(x, y, dx, dy, head_width=0.05, head_length=0.05, fc='blue', ec='blue')
    
    # Annotate with timestamp, formatted to 1 digit after the decimal point
    ax1.annotate(f'{timestamp:.1f}', (x, y), textcoords="offset points", xytext=(5,5), ha='center',fontsize=6)

# Set labels and title for the first subplot
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_title('Bird\'s Eye View of the Robot')
ax1.set_aspect('equal')

# Generate the second plot data
timestamps = np.arange(0, data['timestamp'].max(), 0.1)
x_values = []
y_values = []

for t in timestamps:
    if t % 25 < 5:
        x_values.append(0)
        y_values.append(1)
    elif t % 25 < 10:
        x_values.append(0)
        y_values.append(-1)
    elif t % 25 < 15:
        x_values.append(1)
        y_values.append(0)
    elif t % 25 < 20:
        x_values.append(-1)
        y_values.append(0)
    else:
        x_values.append(1)
        y_values.append(1)

# Plot the second subplot
ax2.plot(timestamps, x_values, label='X')
ax2.plot(timestamps, y_values, label='Y')
ax2.legend()

# Set labels and title for the second subplot
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Value')
ax2.set_title('X and Y Values Over Time')

# Show the plot
plt.show()
# RCLCPP_INFO(this->get_logger(), "0,1");
#             publish_twist_message(0, 1);
#         } else if (elapsed_time < 10) {
#             RCLCPP_INFO(this->get_logger(), "0,-1");
#             publish_twist_message(0, -1);
#         } else if (elapsed_time < 15) {
#             RCLCPP_INFO(this->get_logger(), "1,0");
#             publish_twist_message(1, 0);
#         }else if (elapsed_time < 20) {
#             RCLCPP_INFO(this->get_logger(), "-1,0");
#             publish_twist_message(-1, 0);
#         }else if (elapsed_time < 25) {
#             RCLCPP_INFO(this->get_logger(), "1,1");
#             publish_twist_message(1, 1);
#         }  else {
#             RCLCPP_INFO(this->get_logger(), "0,0");