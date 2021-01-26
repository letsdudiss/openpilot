#!/usr/bin/env python3
import time
import subprocess
import cereal
import cereal.messaging as messaging
ThermalStatus = cereal.log.ThermalData.ThermalStatus
from selfdrive.swaglog import cloudlog
from common.params import Params, put_nonblocking
params = Params()
from math import floor
import re
import os
from common.realtime import sec_since_boot
import psutil

class App():

  def appops_set(self, package, op, mode):
    self.system(f"LD_LIBRARY_PATH= appops set {package} {op} {mode}")

  def pm_grant(self, package, permission):
    self.system(f"pm grant {package} {permission}")

  def set_package_permissions(self):
    if self.permissions is not None:
      for permission in self.permissions:
        self.pm_grant(self.app, permission)
    if self.opts is not None:
      for opt in self.opts:
        self.appops_set(self.app, opt, "allow")

  def __init__(self, app, start_cmd, enabled, permissions, opts):
    self.app = app
    # main activity
    self.start_cmd = start_cmd
    self.ENABLED = enabled
    # app permissions
    self.permissions = permissions
    # app options
    self.opts = opts

    self.is_running = False

  def uninstall_app(self):
    try:
      subprocess.check_output(["pm","uninstall", self.app])
    except subprocess.CalledProcessError as e:
      pass

  def run(self, force=False):
    self.system("pm enable %s" % self.app)  
    self.system(self.start_cmd)
    self.is_running = True

  def kill(self):
    self.system("killall %s" % self.app)
    self.is_running = False

  def system(self, cmd):
    try:
      subprocess.check_output(cmd, stderr=subprocess.STDOUT, shell=True)
    except subprocess.CalledProcessError as e:
      cloudlog.event("running failed",
                     cmd=e.cmd,
                     output=e.output[-1024:],
                     returncode=e.returncode)

  def isRunning(self):
    #Check if this app is running  
    #Iterate over the all the running process
    for proc in psutil.process_iter():
      try:
        # Check if process name contains the given name string.
        if self.app.lower() in proc.name().lower():
          return True
      except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
        pass
    return False

def init_apps(apps):
  apps.append(App(
    "com.sygic.aura",
    "am start -n com.sygic.aura/com.sygic.aura.activity.NaviNativeActivity",
    False, #Change to True to ENABLE Sygic on car start up
    [
      "android.permission.ACCESS_FINE_LOCATION",
      "android.permission.ACCESS_COARSE_LOCATION",
      "android.permission.READ_EXTERNAL_STORAGE",
      "android.permission.WRITE_EXTERNAL_STORAGE",
      "android.permission.RECORD_AUDIO",
    ],
    [],
  ))

def main():
  apps = []
  frame = 0
  init_done = False
  is_onroad_prev = False

  while 1:
    if not init_done:
      if frame >= 10:
        init_done = True
        init_apps(apps)
        #reset frame count after initialisation
        frame = 0
    else:
      #check if we are on road, run waze if on road, i.e ignition detected, kill app if not on road
      is_onroad = params.get("IsOffroad") != b"1"
      #start GPS app right away if ignition change is detected
      if is_onroad and not is_onroad_prev:
        for app in apps:
          if app.ENABLED:
            #Only start app if it is not running
            #if not app.isRunning():
            app.run()

      #Close all apps when we are offroad      
      if not is_onroad:
        for app in apps:
          if app.is_running:
            app.kill()

      #check if app is running and restart it every 30 seconds (.i.e every 30 frames)
      if frame >= 30:
        frame = 0 #reset frame count when it exceeds 30
        for app in apps:
          if app.ENABLED:
            #if app is detected to be dead and we are on-road, restart the app  
            if is_onroad and is_onroad_prev and not app.isRunning():
            #if is_onroad and is_onroad_prev:
              app.run()
      
      #update is_onroad_prev    
      is_onroad_prev = is_onroad      

    frame += 1
    time.sleep(1) #just sleep 1 second

if __name__ == "__main__":
  main()
