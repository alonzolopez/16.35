def test(z):
	try:
		if z == 3:
			raise ValueError('z is 3'.format(z))
		print("success")
	except Exception, arg:
		print arg

test(4)
