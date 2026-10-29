# Header    header               # Includes measurement timestamp and coordinate frame.
uint32    object_id            # object ID
uint16    age                  # total frames from the object occurs
uint8     measurement_status   # track status (0: measured; 1: new; 2: predicted)
uint8     motion_state         # movement state ( 0: static; 1: dynamic)
uint8     existance_confidence # existance confidence
geometry_msgs/Pose              pose #central position x, y, z; roll=0, pitch=0, yaw to quaternion
geometry_msgs/Vector3           dimensions #length, width, height
geometry_msgs/Twist             velocity
geometry_msgs/Twist             acceleration
geometry_msgs/Twist             v2ground #velocity towards ground of x, y, z orientation
geometry_msgs/Pose              pose_nearest #nearestpts x, y, z
uint8     type                 # object type (（0: car 小型汽车; 1: bike 两轮车; 2: pedestrian 行人; 3: truck 大车; 4: signboard 高空物; 5: ground 地面; 6: obstacle 静态障碍物)
uint8     car_confidence       # car confidence
uint8     bike_confidence      # bike confidence
uint8     ped_confidence       # pedestrian confidence
uint8     truck_confidence     # truck confidence
uint8     signboard_confidence # signboard confidence
uint8     ground_confidence    # ground confidence
uint8     obstacle_confidence  # obstacle confidence
float32   enrollptsnum         # EnrollPtsNum
float32   nearestptsx          # nearestptsX           
float32   nearestptsy          # nearestptsY
float32   nearestptsz          # nearestptsZ
float32   reserved_d  
uint32    reserved_e          
uint32    reserved_f           
int32     reserved_g           
int32     reserved_h    
uint16    reserved_i   
uint16    reserved_j    
int16     reserved_k   
int16     reserved_l 
uint8     reserved_m  
uint8     reserved_n    
string    reserved_o
string    reserved_p
