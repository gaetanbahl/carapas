modelName = "HexBeam3d"

loadModel(modelName)
mech = find("models/0")
fem = mech.models().get("fem")
material = fem.getMaterial()

# don't wait for real-time (i.e. go faster if possible)
main.getScheduler().setRealTimeAdvance(False)  

# increment
for i in range(100000, 5000001, 100000):
	reset()
	material.setYoungsModulus(i)
	run(5)
	waitForStop()
	print("done " + str(i))
	# save stuff
