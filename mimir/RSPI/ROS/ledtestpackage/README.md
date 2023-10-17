This is still in testing

This node taking those command 
```bash
rostopic pub /hello std_msgs/String "hello" 
```
```bash
rostopic echo /hello
```
Respond hi


```bash
rostopic pub /light std_msgs/String "on"
```
```bash
rostopic echo /light
```
Respond NONE


```bash
rostopic pub /speed std_msgs/Float64 "data: 5.0"
```
```bash
rostopic echo /speed
```
Respond Received speed: 5.0
