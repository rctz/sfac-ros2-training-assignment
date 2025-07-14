
## Day 1: Battery and Temperature Monitoring

### 🧩 Tasks
---
### 🔹 Step 1. Create Custom Message: `BatteryState.msg`

```
float32 voltage  
float32 current  
float32 percentage  
float32 temperature  
string status # values: "OK", "OVERHEAT", "SHORT" "CRITICAL"  
builtin_interfaces/Time timestamp
```
---
### 🔹 Step 2: Write `battery_state_publisher` Node

- Simulate battery and temperature data, publish every second, and log results.
- Use ROS2 multi-threaded executor to allow flexible callbacks (see ROS2 Multi-threaded Executor ).
- Implement dynamic parameters 

#### ⏱️ Timer Callback (Every 1 Second)

- On each timer callback, perform the following:

#### 🔄 Simulate Battery Data (Randomly):

- **Voltage**: 48V → 40V
- **Current**: 10A → 0A
- **Percentage**: Decrease by **3–8%** per cycle _(simulate battery drain)_
- **Temperature**: 22°C → 70°C  
  - Occasionally spike to simulate faults


#### 🔍 Determine Battery `status`

- Set `status` field in the message according to this logic:

| Condition                             | Status       |
|--------------------------------------|--------------|
| All values within safe range         | `OK`         |
| Temperature > 60°C                   | `OVERHEAT`   |
| Voltage < 42V                        | `SHORT`      |
| Percentage < 15%                     | `CRITICAL`   |

- Add the current **ROS2 time** as `timestamp`

#### 📤 Publish

- Publish the data as a custom `BatteryState` message to topic: `/battery_state`

#### 📝 Logging
- Write every message to a CSV file (battery_log.csv) in this format:
```
timestamp,voltage,current,percentage,temperature,status
```
#### 🔧 Dynamic Parameters (Runtime Configurable)

Enable the ability to **adjust fault thresholds at runtime** using ROS 2 parameters.

#### 📌 Threshold Parameters

Set the following parameters with default values:

| Parameter Name                | Default Value |
|------------------------------|----------------|
| `/overheat_threshold`        | `60.0` (°C)    |
| `/low_voltage_threshold`     | `42.0` (V)     |
| `/critical_percentage_threshold` | `15.0` (%) |

These parameters should be dynamically changeable **without restarting the node**.

- 🛠️ Use: `rclpy.Parameter` or the node's parameter interface
- 📚 Reference: [ROS2 Setting Parameters at Runtime](https://roboticsbackend.com/ros2-rclpy-parameter-callback/)
  

#### 💡 Useful Hints

- ✅ Use `rclpy.Parameter` or `self.declare_parameter()` / `self.get_parameter()` for dynamic parameter handling
- ✅ Support **parameter updates** via `on_set_parameters_callback()` if needed
- ⚠️ Handle **exceptions**:
  - If the CSV file is missing or unwritable, fallback by logging to the terminal
  - Use `try/except` blocks around file writing operations to prevent crashes

---

### 🔍 Step 3: Write `diagnostics_subscriber` Node

Monitor battery health, detect dangerous events, log warnings, and record incident reports.


#### 📥 Subscribe

- Subscribe to the topic: `/battery_state`
- On every message:
  - If status != `"OK"`:
    - Print a warning on the terminal:
    - `WARNING — [timestamp] — Status: [status] — Take Action!`
    - Write an event log CSV: 
      ```
      timestamp,status,action
      ``` 
    - “action” can be: “WATCH”, “SLOW DOWN”, “PARK”
  - If status == `"CRITICAL"`
    - Prepare to call parking action server in Day 2.
- Useful Hints: 
    - Keep the event log and data log separate (event_log.csv only for real incidents).
    - Always include timestamps for traceability.
     
---

### ✅ Step 4: Testing Your Work

1. **Launch both nodes** (`battery_state_publisher` and `diagnostics_subscriber`)
2. **Watch the logs**:
   - Observe `status` values (e.g., `OK`, `OVERHEAT`, `SHORT`, `CRITICAL`)
   - Confirm that faults are being detected and logged
   - Try simulating:
     - Low voltage
     - High temperature
3. **Inspect CSV files**:
   - Is each value logged correctly?
   - Are the timestamps accurate?
   - Do events match the simulated faults?

---
### 💡 Useful Suggestions

- 🔧 **Make thresholds and log file paths configurable**
  - Use ROS 2 parameters to adjust:
    - Overheat voltage
    - Critical percentage
    - Output CSV path
  - Improves **reusability** and **project flexibility**

- 🔁 **Add a parameter to switch simulation ON/OFF**
  - Helps with **automated testing** by disabling random value changes

- 🧑‍💻 **Consult with senior/dev lead** if:
  - Logs **miss expected events**
  - Parameter updates are **not reflected** properly

- 📋 **Regularly check the CSV format**
  - Properly formatted logs are your **best diagnostic tool**


### 📚 References

- [Custom ROS 2 Interfaces: Messages](https://docs.ros.org/en/foxy/How-To-Guides/Custom-ROS2-Interfaces.html)
- [Writing ROS 2 Publishers in Python](https://docs.ros.org/en/foxy/Tutorials/Topics/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 Parameters (Python)](https://docs.ros.org/en/foxy/How-To-Guides/Parameters/Understanding-ROS2-Parameters.html)
- [Multi-threaded Executors](https://docs.ros.org/en/foxy/How-To-Guides/Node-Callbacks/With-Executors.html)
- [Python CSV Logging](https://docs.python.org/3/library/csv.html)

## 💪 Extra Encouragement

> Every faulty battery you catch here could save a car—and maybe a life—in the future.  
> **Give it your best**, and keep your logs strong; your system is the car’s eyes and nerves! 🚗⚡


---