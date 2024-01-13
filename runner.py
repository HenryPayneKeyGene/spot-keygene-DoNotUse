#  Copyright (c) Romir Kulshrestha 2023.

from datetime import datetime

from spotkg.keygene import gui, queue_runner, execution_order

# execution_order.put((datetime(hour=12, minute=0, second=0), ["scan"]))
# queue_runner()
gui()
