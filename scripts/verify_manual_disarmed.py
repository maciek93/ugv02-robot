#!/usr/bin/env python3
"""Socket/manual-page checks with motion disarmed; no successful movement."""
import json
import urllib.request
import socketio
import time
base='http://127.0.0.1:5000'
health=json.load(urllib.request.urlopen(base+'/health',timeout=3))
assert not health['motion_enabled'] and not health['moving_command_active'],health
page=urllib.request.urlopen(base+'/manual',timeout=3).read()
assert b'UGV02 manual control' in page and b'pointercancel' in page
client=socketio.Client()
client.connect(base,namespaces=['/manual'],transports=['polling'])
sid=client.get_sid('/manual')
def movement(seq, age=0, connection=sid):
    return {'left':.08,'right':.08,'duration':.3,'seq':seq,'connection':connection,'sent_at_ms':time.time()*1000-age}
reply=client.call('drive',movement(1),namespace='/manual',timeout=3)
assert reply['ok'] is False and 'disarmed' in reply['error'].lower(),reply
reply=client.call('stop',{'seq':2,'connection':sid},namespace='/manual',timeout=3)
assert reply['ok'] is True and not reply['moving_command_active'],reply
reply=client.call('drive',movement(1),namespace='/manual',timeout=3)
assert reply['ok'] is False and 'stale' in reply['error'].lower(),reply
reply=client.call('drive',movement(3,connection=sid+'-old'),namespace='/manual',timeout=3)
assert reply['ok'] is False and 'previous connection' in reply['error'].lower(),reply
reply=client.call('drive',movement(3,age=1000),namespace='/manual',timeout=3)
assert reply['ok'] is False and 'expired' in reply['error'].lower(),reply
client.disconnect()
print('PASS: manual page, disarmed drive rejection, Stop, stale sequence, expired request and old connection rejection')
