##############################################################
def walkItems(haystack, needle):
	nxt = False

	if isinstance(haystack, dict):
		for key, items in haystack.items():
			if nxt:
				return key

			elif isinstance(items, dict):
				result = walkItems(items, needle)
				if result != "":
					return result

			elif isinstance(items, list):
				result = walkItems(items, needle)
				if result != "":
					return result

			elif key == needle:
				nxt = True

		return ""

	elif isinstance(haystack, list):
		for key in haystack:
			if nxt:
				return key
			elif isinstance(key, dict):

				result = walkItems(key, needle)
				if result != "":
					return result

			elif isinstance(key, list):
				result = walkItems(key, needle)
				if result != "":
					return result

			elif key == needle:
				nxt = True

		return ""

	return ""
##############################################################

print(">---------- PRE Compile Script BEGIN ----------<")

Import("env")

# Dump construction environments (for debug purposes)
# print(env.Dump())

# access to global construction environment
# print(env)

my_flags = env.ParseFlags(env['BUILD_FLAGS'])
my_cppdefines = my_flags.get("CPPDEFINES")

MyName = walkItems(my_cppdefines, 'DeviceName')
MyType = walkItems(my_cppdefines, 'DeviceType')

print("BUILD FLAGS: ")
print(env.ParseFlags(env['BUILD_FLAGS']))
print("=========================================================================================================")
print("PIOENV:\t\t",   env['PIOENV'])
print("BOARD MCU:\t",  env['BOARD_MCU'])
print("FLASH SIZE:\t", walkItems(my_cppdefines, 'FlashSize'))
print("UNIX TIME:\t",  env['UNIX_TIME'])

# print("-------")

print(">---------- PRE Compile Script END ----------<")
