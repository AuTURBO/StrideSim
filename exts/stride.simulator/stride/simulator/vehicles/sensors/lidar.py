__all__ = ["Lidar"]

from omni.isaac.core import World
from stride.simulator.vehicles import State
from stride.simulator.vehicles.sensors.sensor import Sensor

from omni.isaac.sensor import RotatingLidarPhysX
from omni.isaac.range_sensor import _range_sensor               # Imports the python bindings to interact with lidar sensor

class Lidar(Sensor):
    """
    The class that implements the Lidar sensor. This class inherits the base class Sensor.
    """
    
    def __init__(self, config=None):
        """
        Initialize the Lidar class

        Args:
        """
        if config is None:
            config = {}
        else:
            assert isinstance(config, dict), "The config parameter must be a dictionary."
            
        self.config = config
            
        super().__init__(sensor_type="Lidar", update_frequency=self.config.get("update_frequency", 10.0))
                        
        # Save the state of the sensor
        self._state = {
            "points": [],
        }
        
        self.lidar_interface = _range_sensor.acquire_lidar_sensor_interface() # Used to interact with the LIDAR
        
        self.lidar_flag_ = False
        

    @property
    def state(self):
        return self._state
    
    @Sensor.update_at_frequency
    def update(self, state: State, dt: float):
        
        if self.lidar_flag_ is True:
            pointcloud = self.lidar_interface.get_point_cloud_data(self.config.get("prim_path"))
            print(pointcloud)
            self._state['points'] = pointcloud
        
        else:
            my_world = World.instance()
            
            ## check there is world
            if my_world is None:
                pass
            else:
                self._lidar = my_world.scene.add(
                            RotatingLidarPhysX(prim_path=self.config.get("prim_path"), name="range_sensor",
                            rotation_dt = 10
                            )
                )
                self._lidar.set_fov([360, 30])
                self._lidar.set_resolution([0.4, 0.4])
                self._lidar.set_valid_range([0.1, 6])
                self._lidar.enable_visualization(high_lod=True,
                                                 draw_points=True,
                                                 draw_lines=False)
                
                self.lidar_flag_ = True
        
        return self._state