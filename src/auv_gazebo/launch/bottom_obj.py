import random

fl_1 = [random.uniform(24.0,43.15), random.uniform(15.0,24.05)]
fl_2 = [random.uniform(24.0,43.15), random.uniform(15.0,24.05)]
pf = random.sample([fl_1, fl_2], 1)

bc = [random.uniform(24.0,43.15), random.uniform(15.0,24.05)]
pb = [[bc[0] + 1.5, bc[0] + 3.4, bc[0] + 5.75], bc[1]]

my_file = open("bottom_objects.launch", "w")
my_file.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
              "<launch>\n\n"
              
              "    <include file=\"$(find spawn_sdf)/launch/spawn_sdf.launch\">\n"
              "        <arg name=\"robot_name\" value=\"main_gate\" />\n"
             f"        <arg name=\"x\" value=\"{random.uniform(23.0,49.15)}\" />\n"
              "        <arg name=\"y\" value=\"12.0\" />\n"
              "        <arg name=\"z\" value=\"3.2\" />\n"
              "        <arg name=\"roll\" value=\"0\"/>\n"
              "        <arg name=\"pitch\" value=\"0\"/>\n"
              "        <arg name=\"yaw\" value=\"0.0\" />\n"
              "        <arg name=\"sdf_robot_file\" value=\"$(find spawn_sdf)/models/main_gate/model.sdf\" />\n"
              "    </include>\n\n"
              
              "    <include file=\"$(find spawn_sdf)/launch/spawn_sdf.launch\">\n"
              "        <arg name=\"robot_name\" value=\"qualification_gate\" />\n"
             f"        <arg name=\"x\" value=\"{random.uniform(8.0,12.0)}\" />\n"
             f"        <arg name=\"y\" value=\"{random.uniform(4.0,6.0)}\" />\n"
              "        <arg name=\"z\" value=\"3.2\" />\n"
              "        <arg name=\"roll\" value=\"0\"/>\n"
              "        <arg name=\"pitch\" value=\"0\"/>\n"
              "        <arg name=\"yaw\" value=\"1.57079\" />\n"
              "        <arg name=\"sdf_robot_file\" value=\"$(find spawn_sdf)/models/qualification_gate/model.sdf\" />\n"
              "    </include>\n\n"
              
              "    <include file=\"$(find spawn_sdf)/launch/spawn_sdf.launch\">\n"
              "        <arg name=\"robot_name\" value=\"starting_gates\" />\n"
              "        <arg name=\"x\" value=\"35.0\" />\n"
              "        <arg name=\"y\" value=\"0.8\" />\n"
              "        <arg name=\"z\" value=\"3.2\" />\n"
              "        <arg name=\"roll\" value=\"0\"/>\n"
              "        <arg name=\"pitch\" value=\"0\"/>\n"
              "        <arg name=\"yaw\" value=\"0.0\" />\n"
              "        <arg name=\"sdf_robot_file\" value=\"$(find spawn_sdf)/models/starting_gates/model.sdf\" />\n"
              "    </include>\n\n"
              
              "    <include file=\"$(find spawn_sdf)/launch/spawn_sdf.launch\">\n"
              "        <arg name=\"robot_name\" value=\"yellow_flare_1\" />\n"
             f"        <arg name=\"x\" value=\"{fl_1[0]}\" />\n"
             f"        <arg name=\"y\" value=\"{fl_1[1]}\" />\n"
              "        <arg name=\"z\" value=\"3.2\" />\n"
              "        <arg name=\"roll\" value=\"0\"/>\n"
              "        <arg name=\"pitch\" value=\"0\"/>\n"
              "        <arg name=\"yaw\" value=\"0.0\" />\n"
              "        <arg name=\"sdf_robot_file\" value=\"$(find spawn_sdf)/models/yellow_flare/model.sdf\" />\n"
              "    </include>\n\n"
              
              "    <include file=\"$(find spawn_sdf)/launch/spawn_sdf.launch\">\n"
              "        <arg name=\"robot_name\" value=\"yellow_flare_2\" />\n"
             f"        <arg name=\"x\" value=\"{fl_2[0]}\" />\n"
             f"        <arg name=\"y\" value=\"{fl_2[1]}\" />\n"
              "        <arg name=\"z\" value=\"3.2\" />\n"
              "        <arg name=\"roll\" value=\"0\"/>\n"
              "        <arg name=\"pitch\" value=\"0\"/>\n"
              "        <arg name=\"yaw\" value=\"0.0\" />\n"
              "        <arg name=\"sdf_robot_file\" value=\"$(find spawn_sdf)/models/yellow_flare/model.sdf\" />\n"
              "    </include>\n\n"

              "    <include file=\"$(find spawn_sdf)/launch/spawn_sdf.launch\">\n"
              "        <arg name=\"robot_name\" value=\"pinger_buckets\" />\n"
             f"        <arg name=\"x\" value=\"{random.sample(pb[0], 1)}\" />\n"
             f"        <arg name=\"y\" value=\"{pb[1]}\" />\n"
              "        <arg name=\"z\" value=\"1.3\" />\n"
              "        <arg name=\"roll\" value=\"0\"/>\n"
              "        <arg name=\"pitch\" value=\"0\"/>\n"
              "        <arg name=\"yaw\" value=\"0.0\" />\n"
              "        <arg name=\"sdf_robot_file\" value=\"$(find spawn_sdf)/models/pinger/model.sdf\" />\n"
              "    </include>\n\n"

              "    <include file=\"$(find spawn_sdf)/launch/spawn_sdf.launch\">\n"
              "        <arg name=\"robot_name\" value=\"pinger_flare\" />\n"
             f"        <arg name=\"x\" value=\"{pf[0] + 0.2}\" />\n"
             f"        <arg name=\"y\" value=\"{pf[1] + 0.2}\" />\n"
              "        <arg name=\"z\" value=\"1.3\" />\n"
              "        <arg name=\"roll\" value=\"0\"/>\n"
              "        <arg name=\"pitch\" value=\"0\"/>\n"
              "        <arg name=\"yaw\" value=\"0.0\" />\n"
              "        <arg name=\"sdf_robot_file\" value=\"$(find spawn_sdf)/models/pinger/model.sdf\" />\n"
              "    </include>\n\n"
           
              "    <include file=\"$(find spawn_sdf)/launch/spawn_sdf.launch\">\n"
              "        <arg name=\"robot_name\" value=\"buckets\" />\n"
             f"        <arg name=\"x\" value=\"{bc[0]}\" />\n"
             f"        <arg name=\"y\" value=\"{bc[1]}\" />\n"
              "        <arg name=\"z\" value=\"1.21\" />\n"
              "        <arg name=\"roll\" value=\"0\"/>\n"
              "        <arg name=\"pitch\" value=\"0\"/>\n"
              "        <arg name=\"sdf_robot_file\" value=\"$(find spawn_sdf)/models/buckets/model.sdf\" />\n"
              "    </include>\n\n"
              
              "    <include file=\"$(find spawn_sdf)/launch/spawn_sdf.launch\">\n"
              "        <arg name=\"robot_name\" value=\"red_flare\" />\n"
             f"        <arg name=\"x\" value=\"{random.uniform(22.0,49.0)}\" />\n"
             f"        <arg name=\"y\" value=\"{random.uniform(4.0,8.0)}\" />\n"
              "        <arg name=\"z\" value=\"3.2\" />\n"
              "        <arg name=\"roll\" value=\"0\"/>\n"
              "        <arg name=\"pitch\" value=\"0\"/>\n"
              "        <arg name=\"yaw\" value=\"0.0\" />\n"
              "        <arg name=\"sdf_robot_file\" value=\"$(find spawn_sdf)/models/red_flare/model.sdf\" />\n"
              "    </include>\n\n"
              
              "</launch>")
my_file.close()