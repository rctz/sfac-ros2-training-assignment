# 🚗 The E-car Readiness Gauntlet (Basic Level)

## 🎯 Why This Project Matters

After Assignment 3, you are ready to join the real E-car project.  
But this new challenge is **50% tougher**, designed to test your ability as a **leading robotics developer**—not just a follower.  
In these **3 days**, you will simulate a real-world, high-pressure engineering sprint.  
Every part you build gets you (and us!) closer to our deepest goals.

---

## 🛠️ Mission

**Design a smarter E-car subsystem in ROS2.**  
Your subsystem must:

- Communicate between **hardware and app**
- Detect **faults**
- Run **concurrent actions**
- Be **debuggable**
- Handle **failure gracefully**

---

## ✨ Key Features (Upgrades from Basic Version)

- ✅ Use **multi-threading / concurrent callbacks**
- ✅ Parking is done via an **action server** (not just a service)
- ✅ Battery data and events are **logged to CSV** files
- ✅ **Dynamic parameters** can be changed during runtime
- ✅ Simulate additional faults: **battery overheat / short-circuit** with **recovery logic**
- ✅ Basic **automated test** through a ROS2 node (**bonus**)
- ✅ Well-written **README** and **architecture diagram**

---

# ✅ Submission Checklist (Organized by Day)

---

## 🟡 Day 1: Battery and Temperature Monitoring

- [ ] **Custom message defined:** `BatteryState.msg`  
  *(Includes: voltage, current, percentage, temperature, status, timestamp)*

- [ ] **`battery_state_publisher` node implemented**
  - Publishes simulated battery data
  - Supports **dynamic runtime parameter updates**
  - Uses `MultiThreadedExecutor`
  - Logs all data to a **CSV file**, handles exceptions safely

- [ ] **`diagnostics_subscriber` node**
  - Subscribes to `/battery_state`
  - Logs warnings/incidents (when `status ≠ "OK"`) to a separate **event log CSV**
  - Prepares to trigger parking when status is `"CRITICAL"`

- [ ] **Demonstration**
  - Terminal screenshots showing real-time publishing/subscription
  - Snapshots of CSV logs showing valid and error entries

---

## 🔴 Day 2: Action Server & Recovery

- [ ] **Custom action defined:** `Parking.action`  
  *(Goal: mode, Feedback: status/percent_complete, Result: parked/report)*

- [ ] **`parking_action_server` node**
  - Handles both normal and emergency parking modes
  - Sends feedback at regular intervals (every 0.2s in emergency mode)
  - Implements **recovery/cancel logic** if battery status returns to OK
  - Logs every parking attempt (successful or canceled) to a **CSV**
  - Uses `MultiThreadedExecutor` for parallel handling of actions and cancellations

- [ ] **(Bonus)** `fault_injector` node/script  
  *(Simulates transitions like normal → overheat → critical → ok)*

- [ ] **Demonstration**
  - Trigger parking using simulated `"CRITICAL"` status
  - Show recovery/cancellation via logs and terminal
  - Include **terminal screenshots + logs** of all key moments

---

## 🔵 Day 3: System Integration & Validation

- [ ] **Complete launch file**
  - Launches: `battery_state_publisher`, `diagnostics_subscriber`, `parking_action_server`, and optionally `fault_injector`
  - Supports parameter overrides via CLI or YAML config

- [ ] **Test script or node** for automated testing
  - Simulates at least 3 test cases:
    - normal → overheat → critical → recover
    - immediate emergency parking
    - recovery mid-process

- [ ] **Well-structured `README_PROJECT.md`**
  - System architecture diagram (block or node graph)
  - Explanation of:
    - Custom messages/actions
    - Parameters and their effect
    - Fault tolerance and recovery logic
  - Clear **build/run/test instructions**

- [ ] **Supporting artifacts**
  - Screenshots of fault detection, parking trigger, recovery process
  - At least **one CSV data log** and **one event log**

---

## 📦 General (Applies to All Days)

- [ ] All code and artifacts are pushed to your GitHub fork/branch
- [ ] Pull request and documentation submitted for code review
- [ ] **This checklist is completed and included** in the final submission
- [ ] Evidence (logs, screenshots, diagrams) is included to demonstrate correctness

---

## 🔁 Reminder

> ✅ **Show your work clearly.**  
> Every screenshot, log entry, and README paragraph should prove that:
> - Your system works  
> - It handles faults  
> - It recovers gracefully  
> - It's the kind of engineering you'd trust in a real car.  


---

# Motivation & Encouragement

Mistakes are not failures—they’re proof you’re learning. Debug fiercely. Every obstacle you overcome here proves you are ready not just for the E-car, but for any hard mission in life.  
What you build now will directly help our team, our families, and those we care for most.

---

# FAQ

### Q: What’s a Multi-Threaded Executor?  
**A:** It lets a node run multiple callback functions in parallel. In real E-cars, many things happen at once—this gives you that power in simulation.

### Q: Where do I save error/fault logs?  
**A:** In CSV files. Clearly document their location and format in your README.
