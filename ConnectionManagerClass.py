import sys

# Class-use constants:
_OUTER_BROKER_PORT = 8000
_INNER_BROKER_ADDRESS = "localhost"
_INNER_BROKER_PORT = 1884

# Apps of the DEE used by the ConnectionManager to retrieve the broker addresses: all lowercase no spaces
_DEE_APPLICATIONS = {'dashboard', 'autopilotservice'}
# Brokers that require username and password
_PROTECTED_BROKERS = {'classpip.upc.edu'}


def processScriptParameters():
    # setParameters SYNTAX: operation_mode (local_mode) (external_broker_address) (user pwd)
    # parent_application (max_drones)
    script_parameters = sys.argv[1:]
    _index = 0
    _index_max = len(script_parameters)

    _operation_mode = script_parameters[_index]
    _index += 1
    _default_ending_arguments = 2
    try:
        _max_drones = int(script_parameters[-1])
        if not _max_drones > 0:
            raise Exception("ERROR: maximum drones must be > 0")
    except ValueError:
        _max_drones = None
        _default_ending_arguments = 1
    if _operation_mode == "local":
        try:
            _local_mode = int(script_parameters[_index])
        except ValueError:
            raise Exception("ERROR: local mode must be either 0 (onboard broker) or 1 (single broker)")
        _index += 1
    else:
        _local_mode = None
    _parent_application = script_parameters[-_default_ending_arguments]
    if _parent_application.lower() not in _DEE_APPLICATIONS:
        raise Exception("ERROR: invalid parent application")
    _external_broker_address = script_parameters[_index: _index_max - _default_ending_arguments]
    _broker_credentials = None

    if _external_broker_address[0] in _PROTECTED_BROKERS:
        _broker_username = script_parameters[-_default_ending_arguments-2]
        _broker_pwd = script_parameters[-_default_ending_arguments-1]
        if _broker_username == _external_broker_address[0] or _broker_pwd == _external_broker_address[0]:
            raise Exception("ERROR: missing broker credentials")
        else:
            _broker_credentials = (_broker_username, _broker_pwd)
        _external_broker_address = _external_broker_address[:-2]
    if len(_external_broker_address) == 3 and _operation_mode == 'global':
        raise Exception(f"ERROR: Too many arguments: {_external_broker_address[1:]}")

    _return_args = (_operation_mode, _parent_application)
    _return_kwargs = {}
    if _local_mode is not None:
        _return_kwargs['local_mode'] = _local_mode
    if _max_drones is not None:
        _return_kwargs['max_drones'] = _max_drones
    if _external_broker_address is not None:
        _return_kwargs['external_broker_address'] = _external_broker_address
    if _broker_credentials is not None:
        _return_kwargs['broker_credentials'] = _broker_credentials

    return _return_args, _return_kwargs


class ConnectionManager:
    def __init__(self):
        # Operation mode: specifies the type of connection:
        # 0: Global mode
        # 11: Local mode, onboard broker
        # 12: Local mode, single broker
        # 2: Direct mode
        # 3: Simulation mode
        # -1: invalid / not set
        self.operation_mode = -1

        # Max drones: the maximum number of drones flying.
        # 1: regular operation
        # > 1: swarm operation
        # -1: invalid / not set
        self.max_drones = -1

        # Parent application: the app that uses this class
        self.parent_application = ""

        # Outer broker address and port
        self.outer_broker_address = ("",)
        self.outer_broker_port = -1

        # Broker credentials for the classpip.upc.edu broker
        self.broker_credentials = (None, None)

        # Inner broker address and port
        self.inner_broker_address = ""
        self.inner_broker_port = -1

    def setParameters(self, operation_mode, parent_application, local_mode=-1, max_drones=1,
                      external_broker_address=None, broker_credentials=None):
        self.__setOperationMode(operation_mode, local_mode)
        self.__setMaxDrones(max_drones)
        self.__setParentApplication(parent_application)
        if external_broker_address is not None:
            self.__setExternalAddress(external_broker_address)
        if broker_credentials is not None:
            self.__setBrokerCredentials(broker_credentials)
        return self.__getOuterAddress(), self.__getInnerAddress()

    def __setOperationMode(self, operation_mode, local_mode=-1):
        # Operation mode
        if operation_mode == "global":
            self.operation_mode = 0
        elif operation_mode == "local" and local_mode >= 0:
            if local_mode == 0:
                self.operation_mode = 11
            elif local_mode == 1:
                self.operation_mode = 12
            else:
                return Exception("ERROR: Invalid local mode (0 = Onboard broker, 1 = single broker)")
        elif operation_mode == "direct":
            self.operation_mode = 2
        elif operation_mode == "simulation":
            self.operation_mode = 3
        else:
            return Exception("ERROR: Invalid operation mode (global, local, direct, simulation)")

    def __setMaxDrones(self, max_drones):
        # Max drones
        if (type(max_drones) is not int) or max_drones < 1:
            return Exception("ERROR: Max drones must be a valid number (int > 0)")
        else:
            self.max_drones = max_drones

    def __setParentApplication(self, parent_application):
        # Parent application
        if parent_application.lower() == "dashboard":
            self.parent_application = "dashBoard"
        elif parent_application.lower() == "autopilotservice":
            self.parent_application = "autopilotService"
        else:
            return Exception("ERROR: Invalid parent application (either dashBoard or autopilotService)")

    def __setExternalAddress(self, external_broker_address):
        # MQTT addresses
        if self.parent_application == "dashBoard":
            if self.operation_mode == 0:
                self.outer_broker_address = external_broker_address
                self.outer_broker_port = _OUTER_BROKER_PORT
            elif self.operation_mode == 11:
                self.outer_broker_address = external_broker_address
                self.outer_broker_port = _OUTER_BROKER_PORT
            else:
                self.outer_broker_address = _INNER_BROKER_ADDRESS
                self.outer_broker_port = _OUTER_BROKER_PORT
        elif self.parent_application == "autopilotService":
            self.inner_broker_address = _INNER_BROKER_ADDRESS
            self.inner_broker_port = _INNER_BROKER_PORT
            if self.operation_mode == 0:
                self.outer_broker_address = external_broker_address
                self.outer_broker_port = _OUTER_BROKER_PORT
            elif self.operation_mode == 12:
                self.outer_broker_address = external_broker_address
                self.outer_broker_port = _OUTER_BROKER_PORT
            else:
                self.outer_broker_address = _INNER_BROKER_ADDRESS
                self.outer_broker_port = _OUTER_BROKER_PORT

    def __setBrokerCredentials(self, broker_credentials):
        if self.outer_broker_address in _PROTECTED_BROKERS:
            self.broker_credentials = broker_credentials
        else:
            return Exception(f"ERROR: no credentials supported for {self.outer_broker_address}")

    def __getOuterAddress(self):
        if self.outer_broker_address and self.outer_broker_port > 0:
            if self.broker_credentials != (None, None):
                return self.outer_broker_address, self.outer_broker_port, self.broker_credentials
            else:
                return self.outer_broker_address, self.outer_broker_port
        else:
            return None

    def __getInnerAddress(self):
        if self.inner_broker_address and self.inner_broker_port > 0:
            return self.inner_broker_address, self.inner_broker_port
        else:
            return None


if __name__ == "__main__":
    print(f"system arguments: {sys.argv[1:]}")
    # PARAM SYNTAX: operation_mode (local_mode) (external_broker_address) (username password)
    # parent_application (max_drones)
    # FUNCTION SYNTAX: operation_mode, parent_application, local_mode, max_drones, external_broker_address
    processed_parameters = processScriptParameters()
    print(f'processed parameters: {processed_parameters}')
    testCon = ConnectionManager()
    if not processed_parameters == Exception:
        print(f'connection manager output: {testCon.setParameters(*processed_parameters[0], **processed_parameters[1])}')
