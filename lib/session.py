import signal
import asyncio
from lib.exceptions import *


class Session:
    """Contains asyncio tasks, run the main asyncio loop, and contains shared objects"""

    def __init__(self):
        self.tasks = []
        self.loop = asyncio.get_event_loop()

    def start(self):
        """Run starting behavior. Override to make it do something useful"""
        pass

    def run(self):
        """Run all added tasks until one raises an exception or they all finish"""
        self.start()

        # add detection for KeyboardInterrupt
        for sig in (signal.SIGINT, signal.SIGTERM):
            self.loop.add_signal_handler(sig, self.ask_exit)

        exception = None
        try:
            # run all added asyncio tasks
            self.loop.run_until_complete(asyncio.gather(*self.tasks))
        except SessionFinishedException as e:
            exception = e
            self.ask_exit_and_wait()
            print("SessionFinishedException raised. Exiting")
        except asyncio.CancelledError:
            pass
        except BaseException as e:
            exception = e
            raise
        finally:
            # run shutdown tasks
            for sig in (signal.SIGINT, signal.SIGTERM):
                self.loop.remove_signal_handler(sig)
            self.stop(exception)
            self.loop.close()

    def stop(self, exception):
        """Run stopping behavior. Override to make it do something useful"""
        pass

    def add_task(self, coroutine):
        """Starts a coroutine by calling asyncio.ensure_future on it"""
        task = asyncio.ensure_future(coroutine, loop=self.loop)
        self.tasks.append(task)

    def ask_exit(self):
        """Request all added tasks be cancelled"""
        for task in self.tasks:
            task.cancel()

    def ask_exit_and_wait(self):
        """Callback for when a task raises an SessionFinishedException. Cancels tasks and waits for them to finish"""
        self.ask_exit()
        while not all([t.done() for t in self.tasks]):
            self.loop.run_until_complete(asyncio.sleep(0.0))
