object Docker {

  val akkaTcpPort = 2551

  val bashExports = """
MAX_CLUSTER_SEEDS=3

# Configure cluster seeds from service if passed
# otherwise keep the seeds as they are
if [ ! -z "$CLUSTER_DISCOVERY_SERVICE" ]; then
  CLUSTER_SEEDS=$(
    host -t A $CLUSTER_DISCOVERY_SERVICE | \
    grep 'has address' |
    head -n $MAX_CLUSTER_SEEDS |
    cut -d ' ' -f 4 |
    xargs |
    sed -e 's/ /,/g'
  )
  if [ ! -z "$CLUSTER_SEEDS" ]; then
    export CLUSTER_SEEDS
  fi
fi

export REMOTE_INTERFACE=$(hostname --ip-address)

# Configure the default cluster port unless passed
if [ -z "$REMOTE_PORT" ]; then
  export REMOTE_PORT=""" + akkaTcpPort + """
fi
"""

}
