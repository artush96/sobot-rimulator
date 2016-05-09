#!/usr/bin/env python
import gtk
import gobject

from models.map_manager import *
from models.robot import *
from models.world import *

REFRESH_RATE = 20.0 # hertz

class IRobotController:
   def __init__( self ):

      self.map_manager = MapManager()

      #timing period
      self.period = 1.0 / REFRESH_RATE

      self.sim_event_source = gobject.idle_add( self.initialize_sim, True )

      #start gtk
      #gtk.main()

      pass

   def initialize_sim( self , random = False):
      #create the simulation world
      self.world = World( self.period )
      
      #create the robot
      self.robot = Robot()
      self.world.add_robot( self.robot )

      
      #using a map
      #self.map_manager.random_map( self.world )
      self.map_manager.load_map('maps/cake')
      goal = [0.75, 0.75]
      self.map_manager.set_goal(goal)
      self.map_manager.apply_to_world(self.world)

   def play_sim( self ):
      gobject.source_remove( self.sim_event_source )
      self._run_sim()
      
   def end_sim( self, alert_text='' ):
      self.robot.supervisor._stop_robot()
      gobject.source_remove( self.sim_event_source )
      
   def _run_sim( self ):
      self.sim_event_source = gobject.timeout_add( int( self.period * 1000), self._run_sim )
      self._step_sim()

   def _step_sim( self ):
      #increment the simulation
      try:
         self.world.step()
      except CollisionException:
         self.end_sim('Collision')
      except GoalReachedException:
         self.end_sim('Goal reached!')


iRb = IRobotController()

iRb.initialize_sim()

try:
   while(1):
      iRb.play_sim()

except:
   iRb.end_sim('Goal reached')

