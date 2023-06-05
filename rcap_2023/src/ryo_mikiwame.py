#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
import rospy
import smach
import smach_ros

#hsrb_interface
import hsrb_interface
from hsrb_interface import Robot, geometry

sys.path.append("/home/demulab/dspl_ws/src/hsr/robot/src")
#from module import *
from arm import *
from humanmodule import *
from audiomodule import *

robot = hsrb_interface.Robot()
tts = robot.get("default_tts")

#locatin_list 辞書型で定義
location_list = {'table_a':(2.284162159336223, 2.929703496804872, 0.061572093333072314),
                 'table_b':(2.380020627581534, 1.056500281377312, 0.02266108632762348),
                 'table_center':(1.5827454106605983, 1.8262875841168187, 3.129996744694669),
                 'table_left':(1.595966813767852, 1.2583708350487552, 3.120435860989875),
                 'table_right':(1.563144172541887, 2.383551753188647, 3.140204424235087),
                 'waypoint1':(2.439885828485936, 0.12257944264739871, 0.0452156824171235),
                 'waypoint2':(3.9091511519156223, 0.15497955104277086, 0.038638968574600786),
                 'shelf':(4.4331769989766485, 1.0486846497559235, 0.03786785857847326),
                 'human_center':(4.63224620012862, 1.4850550571247705, 1.609544631220165),
                 'human_left':(4.022050393438807, 2.2987666211105346, 1.6000028501606836),
                 'human_right':(5.053066552752714, 2.347473880985127, 1.5918783317735565)}

class State1():
	def __init__(self):
		print("dspl")
		self.Do_list = []
		self.count = 0
	def voice(self,self.count):
		self.Do_list = Gpsr.Voice()
		if self.Do_list[0] == "Go":
			for a in range(len(self.Do_list)):
				if self.Do_list[a] == "grasp":
					for b in range(len(self.Do_list)):
						if self.Do_list[b] == "place":
							place(self.Do_list,self.count)
						elif self.Do_list[b] == "give":
							give(self.Do_list,self.count)
						else:
							tts.say("すみません、もう一度お願いします")
							rospy.sleep(1)
							voice()
				elif self.Do_list[a] == "find":
					for c in range(len(self.Do_list)):
						if self.Do_list[c] == "follow":
							follow(self.Do_list,self.count)
						elif self.Do_list[c] == "guide":
							guide(self.Do_list,self.count)
						elif self.Do_list[c] == "answer":
							answer(self.Do_list,self.count)
						elif self.Do_list[c] == "ask":
							ask(self.Do_list,self.count)
						else:
							tts.say("すみません、もう一度お願いします")
							voice()
				else:
					tts.say("すみません、もう一度お願いします")
					voice()
		elif self.Do_list[0] == "Tell"
	
	def place(self,self.Do_list,self.count):
		#placeのときのコードを書く
		for d in range(11):
			if location_list[d][0] == self.Do_list[1]:
				omnibase.go(location_list[d][0],location_list[d][1],location_list[d][2])
				#ものを掴む
				for e in range(11):
					if location_list[e][0] == self.Do_list[7]:
						omnibase.go(location_list[e][0],location_list[e][1],location_list[e][2])
				#ものを置く
			else:
				pass
		self.count =+ 1
		main()
	def give(self,self.Do_list,self.count):
		#giveのときのコードを書く
		for f in range(11):
			if location_list[f][0] == self.Do_list[1]:
				omnibase.go(location_list[f][0],location_list[f][1],location_list[f][2])
				#ものを掴む
				#目的の人に手を上げてもらい移動する
			else:
				pass
		self.count =+ 1
		main()
	def follow(self,self.Do_list,self.count):
		#followのときのコードを書く
		for g in range(11):
			if location_list[g][0] == self.Do_list[1]:
				omnibase.go(location_list[g][0],location_list[g][1],location_list[g][2])
				#目的の人を探す
			else:
				pass
		self.count =+ 1
		main()
	def guide(self,self.Do_list,self.count):
		#guideのときのコードを書く
		for h in range(11):
			if location_list[h][0] == self.Do_list[1]:
				omnibase.go(location_list[h][0],location_list[h][1],location_list[h][2])
				#目的の人を探し移動する
				for i in range(11):
					if location_list[i][0] == self.Do_list[7]:
						omnibase.go(location_list[i][0],location_list[i][1],location_list[i][2])
					else:
						pass
			else:
				pass
		self.count =+ 1
		main()
	def answer(self,self.Do_list,self.count):
		#answerのときのコードを書く
		for j in range(11):
			if location_list[j][0] == self.Do_list[1]:
				omnibase.go(location_list[j][0],location_list[j][1],location_list[j][2])
				#目的の人を探し移動する
				#質問を聞く
			else:
				pass
		self.count =+ 1
		main()
	def ask(self,self.Do_list,self.count):
		#askのときのコードを書く
		for k in range(11):
			if location_list[k][0] == self.Do_list[1]:
				omnibase.go(location_list[k][0],location_list[k][1],location_list[k][2])
				#目的の人を探し移動する
				#質問をする
			else:
				pass
		self.count =+ 1
		main()

	def main(self):
		tts.say("start GPSR")
		rospy.sleep(2)
		if self.count == 3:
			tts.say("end")
			pass
		else:
			voice()

if __name__ == "__main__":
	State1.main()
