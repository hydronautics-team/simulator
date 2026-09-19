import re

sdf = open('/home/pilushok/dev/simulator/install/simulator_description/share/simulator_description/models/rexrov/model.sdf').read()
m = re.search(r'<model[^>]*>.*?</model>', sdf, re.S)
model = m.group(0)
world = ('<?xml version="1.0"?>\n'
         '<sdf version="1.9">\n'
         '  <world name="inline_test">\n'
         '    <physics name="default" type="ode"><gravity>0 0 -9.8</gravity></physics>\n'
         + model +
         '\n  </world>\n</sdf>\n')
open('/home/pilushok/dev/simulator/.codewhale/stage/inline_rexrov.sdf', 'w').write(world)
print("inline world written, model len:", len(model))
