Import("env")
env['PROJECT_SRC_DIR'] = env['PROJECT_DIR'] + "/examples/" + env["PIOENV"]
print("Setting the project directory to: {}".format(env['PROJECT_SRC_DIR']))
