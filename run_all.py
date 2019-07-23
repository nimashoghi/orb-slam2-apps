names = [
    "MH01",
    "MH02",
    "MH03",
    "MH04",
    "V101",
    "V102",
    "V103",
]

command = "slam_app /root/ORBvoc.txt "
for name in names:
    command += " %s both http://192.168.0.113:8080/euroc_mono.yaml http://192.168.0.113:8080/euroc_stereo.yaml http://192.168.0.113:8080/euroc_%s/cam0/data http://192.168.0.113:8080/euroc_%s/cam1/data http://192.168.0.113:8080/euroc_%s/timestamps.txt" % (name, name, name, name)
print(command)
