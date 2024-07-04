## Remote Dataflow

To run remote dataflow

```bash
dora destroy
dora coordinator &
dora daemon --machine-id A &
dora daemon --machine-id B --local-listen-port 53292 &

dora start remote_dataflow.yml
cat timer.csv
```
