## 🚗 Day 2: Action Server & Recovery (with Multithreaded Execution)

### 📋 Tasks
---

### 🔹 Step 1: Define Action Interface: `Parking.action`

```action
# Goal
string mode  # "normal" or "emergency"

---
# Result
bool parked
string report

---
# Feedback
string status
int32 percent_complete
```
----
### 🔹 Step 2: Write parking_action_server Node (with MultiThreadedExecutor) 
#### Runs as a ROS2 node using a MultiThreadedExecutor  
  - This lets the node process multiple callback types in parallel, such as:
    - Handling incoming parking goals (from diagnostics_subscriber or test nodes)
    - Publishing feedback quickly to clients every 0.2 sec, even while handling new requests or cancellation
    - Reacting instantly to cancellation requests or status updates (e.g., if the battery recovers to "OK" and the park goal should be canceled)
             
  - Why is MultiThreadedExecutor needed?  
    - In real applications, actions and feedback/cancel callbacks can come in at the same time.  
    - Multi-threading ensures no callback is blocked, so the server is always responsive.
             
  - Example (Python):
    ```
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    # ... (your node class here)
    def main(args=None):
        rclpy.init(args=args)
        node = ParkingActionServerNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            node.destroy_node()
            rclpy.shutdown()
    ```
#### Action Server Behaviors: `parking_action_server`

  - Receives a parking goal with a specified `mode` ("normal" or "emergency").
  - Example: Simple Parking Sequence (for Parking Action) 
    ```
    - 1.Start Parking:
    - Receive the parking command and check if the system is ready. 
    - 2.Align Vehicle:  
    - Adjust the car to face the parking slot. 
    - 3.Move into Slot:
    - Slowly drive or reverse into the parking spot. 
    - 4.Adjust Position:
    - Fine-tune the car’s position (if needed). 
    - 5. Parking Complete:
    - Stop the vehicle safely in the parking spot and report success. 
    ```
- Example Feedback Sequence 
    - During parking, the action server can send status feedback such as: 
        ```
        "Aligning to parking spot... 20%"
        "Moving into slot... 60%"
        "Adjusting position... 90%"
        "Parking complete! 100%"
        ```

  - If an emergency or cancel request occurs (e.g., battery recovers to OK):   

       "Parking canceled. Vehicle is safe."

- In "normal" mode:
  - Server sends feedback every 0.5 seconds via the feedback channel.

- In "emergency" mode:
  - Parking executes faster (simulated or real).
  - Server sends feedback every 0.2 seconds via the feedback channel.

- On successful completion:
  - Sends a result back to the client with:
    - `report` string
    - Final parking status (`parked: true/false`)

- Recovery Logic:
  - If a new battery state is received during parking and the `status == "OK"`:
    - Cancel the parking goal immediately
    - Notify the client of the cancellation

- Logging Requirement:
  - Every parking action (whether started, canceled, or completed) must be recorded in a CSV file  
    Log format:  
    `timestamp, mode, state (started/completed/canceled), result/report`
----
### 🔹 Step 3: Write fault_injector Node (Bonus Task)
- Publishes abnormal battery or temperature values to trigger faults and test the system’s recovery during action execution.
- Can be triggered by a ROS2 service call (for test automation or demo).
---

### Why Use `MultiThreadedExecutor` in `parking_action_server`?

Actions in ROS 2 (like parking) can involve multiple, concurrent ROS callbacks:

- **Goal callback** – receiving new action requests  
- **Feedback timer** – sending feedback periodically (e.g., every 0.2 seconds)  
- **Cancel callback** – responding to cancellation (e.g., battery state returns to `"OK"`)  
- **Parameter updates** – allowing runtime tuning (for future expansion)

In a **single-threaded executor**, if the server is performing a long task (e.g., sleeping or processing),
other events like cancellation or parameter changes may be **delayed**, potentially causing **unsafe behavior**.

#### MultiThreadedExecutor ensures:

- Action **feedback remains timely** (every 0.2 seconds)
- Server can **immediately respond to cancel** requests (e.g., emergency stop)
- Node behaves in a **robust, responsive**, and **realistic** manner — similar to behavior expected in real-world robots

---

### Tips and Useful References

- ✅ **Recommended:**  
  Subscribe to battery status *in this node* or use a shared, thread-safe variable  
  to detect when the battery has recovered (for cancel/recovery logic)

- ✅ **Logging Tips:**  
  Always include:
  - Timestamp  
  - Goal type  
  - Action outcome  
  For clear debugging and traceability

- 📚 **Learning Resources:**
  - ROS 2 Action Server & Client Tutorial:
  (https://www.theconstruct.ai/ros2-how-to-2-create-a-ros2-action-server/)
---

### In Summary

The `parking_action_server` **must use** `MultiThreadedExecutor` to:

- Handle action goals, feedback, and cancel requests **simultaneously**
- Ensure **realistic and safe** E-car behavior
- React instantly to life-saving conditions — real missions can’t wait for one callback to finish
