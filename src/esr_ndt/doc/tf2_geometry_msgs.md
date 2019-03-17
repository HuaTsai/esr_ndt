* `fromMsg()`
  * `geometry_msgs::Vector3`                     ->    `tf2::Vector3`
  * `geometry_msgs::Vector3Stamped`              ->    `geometry_msgs::Vector3Stamped`
  * `geometry_msgs::Vector3Stamped`              ->    `tf2::Stamped<tf2::Vector3>`
  * `geometry_msgs::Point`                       ->    `tf2::Vector3`
  * `geometry_msgs::PointStamped`                ->    `geometry_msgs::PointStamped`
  * `geometry_msgs::PointStamped`                ->    `tf2::Stamped<tf2::Vector3>`
  * `geometry_msgs::Quaternion`                  ->    `tf2::Quaternion`
  * `geometry_msgs::QuaternionStamped`           ->    `geometry_msgs::QuaternionStamped`
  * `geometry_msgs::QuaternionStamped`           ->    `tf2::Stamped<tf2::Quaternion>`
  * `geometry_msgs::Pose`                        ->    `tf2::Transform`
  * `geometry_msgs::PoseStamped`                 ->    `geometry_msgs::PoseStamped`
  * `geometry_msgs::PoseStamped`                 ->    `tf2::Stamped<tf2::Transform>`
  * `geometry_msgs::PoseWithCovarianceStamped`   ->    `geometry_msgs::PoseWithCovarianceStamped`
  * `geometry_msgs::PoseWithCovarianceStamped`   ->    `tf2::Stamped<tf2::Transform>`
  * `geometry_msgs::Transform`                   ->    `tf2::Transform`
  * `geometry_msgs::TransformStamped`            ->    `geometry_msgs::TransformStamped`
  * `geometry_msgs::TransformStamped`            ->    `tf2::Stamped<tf2::Transform>`
  * `geometry_msgs::WrenchStamped`               ->    `geometry_msgs::WrenchStamped`
  * `geometry_msgs::WrenchStamped`               ->    `tf2::Stamped<boost::array<tf2::Vector3, 2>>`

* `toMsg()`
  * `geometry_msgs::Vector3`                     <-    `tf2::Vector3`
  * `geometry_msgs::Vector3Stamped`              <-    `geometry_msgs::Vector3Stamped`
  * `geometry_msgs::Vector3Stamped`              <-    `tf2::Stamped<tf2::Vector3>`
  * `geometry_msgs::Point`                       <-    `tf2::Vector3` ...
  * `geometry_msgs::PointStamped`                <-    `geometry_msgs::PointStamped`
  * `geometry_msgs::PointStamped`                <-    `tf2::Stamped<tf2::Vector3>` ...
  * `geometry_msgs::Quaternion`                  <-    `tf2::Quaternion`
  * `geometry_msgs::QuaternionStamped`           <-    `geometry_msgs::QuaternionStamped`
  * `geometry_msgs::QuaternionStamped`           <-    `tf2::Stamped<tf2::Quaternion>`
  * `geometry_msgs::Pose`                        <-    `tf2::Transform` ...
  * `geometry_msgs::PoseStamped`                 <-    `geometry_msgs::PoseStamped`
  * `geometry_msgs::PoseStamped`                 <-    `tf2::Stamped<tf2::Transform>` ...
  * `geometry_msgs::PoseWithCovarianceStamped`   <-    `geometry_msgs::PoseWithCovarianceStamped`
  * `geometry_msgs::PoseWithCovarianceStamped`   <-    `tf2::Stamped<tf2::Transform>` ...
  * `geometry_msgs::Transform`                   <-    `tf2::Transform`
  * `geometry_msgs::TransformStamped`            <-    `geometry_msgs::TransformStamped`
  * `geometry_msgs::TransformStamped`            <-    `tf2::Stamped<tf2::Transform>`
  * `geometry_msgs::WrenchStamped`               <-    `geometry_msgs::WrenchStamped`
  * `geometry_msgs::WrenchStamped`               <-    `tf2::Stamped<boost::array<tf2::Vector3, 2>>`

* `tf2::transformTF2ToMsg()`: cannot use
  * `tf2::Transform`, msg
  * `tf2::Quaternion`, `tf2::Vector3`, msg
  * `tf2::Transform`, msg, `ros::Time`, `frame`, `child_frame`
  * `tf2::Quaternion`, `tf2::Vector3`, msg, `ros::Time`, `frame`, `child_frame`

