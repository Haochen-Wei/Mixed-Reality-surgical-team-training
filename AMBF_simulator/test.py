from ambf_client import Client

c=Client()

c.connect()

print(c.get_obj_names())



obj = c.get_obj_handle('/ambf/env/psm1/toolrolllink')

input("wait")
obj.set_pos(0, 0, 0)
obj.set_rpy(1.5, 0.7, .0)


input("wait")
obj.set_pos(0, 0, 0)
obj.set_rpy(0, 0, 0)

input("wait")
obj.set_pos(0, 0, 0)
obj.set_rpy(1, 0, 0)

input("wait")

c.clean_up()