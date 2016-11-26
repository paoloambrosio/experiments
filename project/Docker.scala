object Docker {

  val akkaTcpPort = 2551

  val bashExports = """
export REMOTE_INTERFACE=$(hostname --ip-address)

# Configure the default remote port unless passed
if [ -z "$REMOTE_PORT" ]; then
  export REMOTE_PORT=""" + akkaTcpPort + """
fi
"""

  val frontendBashExports = """
# Configure backend nodes from service if passed
# otherwise keep them as they are
if [ ! -z "$BACKEND_DISCOVERY_SERVICE" ]; then
  BACKEND_NODES=$(
    host -t A $BACKEND_DISCOVERY_SERVICE | \
    grep 'has address' |
    cut -d ' ' -f 4 |
    xargs |
    sed -e 's/ /,/g'
  )
  if [ ! -z "$BACKEND_NODES" ]; then
    export BACKEND_NODES
  fi
fi

""" + bashExports

}
