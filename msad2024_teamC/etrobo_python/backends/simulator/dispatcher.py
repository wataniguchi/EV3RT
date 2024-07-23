import os
from subprocess import DEVNULL, PIPE, Popen
from typing import Any, Callable, List, Optional, Tuple

from etrobo_python.device import Device
from etrobo_python.log import LogWriter

from .connector import connect_simulator


def create_dispatcher(
    devices: List[Tuple[str, Any]],
    handlers: List[Callable[..., None]],
    interval: float = 0.01,
    course: str = 'left',
    timeout: float = 5.0,
    logfile: Optional[str] = None,
    **kwargs,
) -> Any:
    return Dispatcher(
        devices=devices,
        handlers=handlers,
        interval=interval,
        course=course,
        timeout=timeout,
        logfile=logfile,
    )


class Dispatcher(object):
    def __init__(
        self,
        devices: List[Tuple[str, Device]],
        handlers: List[Callable[..., None]],
        interval: float,
        course: str,
        timeout: float,
        logfile: Optional[str],
    ) -> None:
        self.devices = devices
        self.handlers = handlers
        self.interval = interval
        self.course = course
        self.timeout = timeout
        self.logfile = logfile

    def dispatch(self) -> None:
        variables = {name: device for name, device in self.devices}

        writer: Optional[LogWriter] = None
        if self.logfile is not None:
            writer = LogWriter(self.logfile, self.devices)

        def run_handlers():
            for handler in self.handlers:
                handler(**variables)

            if writer is not None:
                writer.write([device for _, device in self.devices])

        connect_simulator(
            handler=run_handlers,
            interval=self.interval,
            address=_get_remote_address(),
            course=self.course,
            timeout=self.timeout,
        )

        if writer is not None:
            writer.close()


def _get_remote_address() -> str:
    '''シミュレータへの通信するためのIPアドレスを返す。

    Returns:
        シミュレータのIPアドレス。
    '''
    # WSL以外の場合はループバックアドレスを返す
    if not os.path.isfile('/proc/sys/fs/binfmt_misc/WSLInterop'):
        return '127.0.0.1'

    # WSLの場合はデフォルトルートのアドレスを返す
    # 通常は、ホストOSのIPアドレスがデフォルトルートになっているため
    pipe = Popen(['ip', 'route'], stdout=PIPE, stderr=DEVNULL)
    outputs = pipe.communicate()[0].decode('utf-8')

    for output in outputs.split('\n'):
        tokens = output.split()
        if len(tokens) < 3:
            continue

        if tokens[0] == 'default' and tokens[1] == 'via':
            return tokens[2]

    # デフォルトルートが見つからない場合はループバックアドレスを返す
    return '127.0.0.1'
