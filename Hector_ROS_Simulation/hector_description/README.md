## How to convert Hector xacro to urdf
Since IsaacSim can only convert urdf to usd, you need to convert xacro to urdf. \
To do so, you  first need to remove xacro-specific line in robot.xacro. 

```xml
<!-- Debug mode will hung up the robot, use "true" or "false" to switch it. -->
<xacro:if value="$(arg DEBUG)">
    <link name="world"/>
    <joint name="base_static_joint" type="fixed">
        <origin rpy="0 0 0" xyz="0 0 1"/>
        <parent link="world"/>
        <child link="base"/>
    </joint>
</xacro:if>
```

Then, build `hector_description` package, and the run the following:
```bash
rosrun xacro xacro -o robot.urdf robot.xacro
```