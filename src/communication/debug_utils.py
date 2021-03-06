import sys
import os
import json

# Finds the root of the proto2 directory
cwd = os.getcwd ().split ("/")
proto_base_path = "/".join (cwd[0:cwd.index ("Proto3") + 1])

class debugging:
	def __init__(self):
		try:
			self.settings = json.load (open (proto_base_path + "/Global_Settings.json", "r"))
		except:
			sys.stderr.write ("Global_Settings.json does not exist or is not in json format!\n")
			sys.exit ()

		if self.settings["Debug"]:
			print "DEBUG = True"

    # Summer: Not bad but could be improved
	def print_d (self, msg):
		if self.settings["Debug"]:
			sys.stderr.write (str (msg) + "\n")
			print str (msg)
