# PID tuning tool

It includes some useful rules for PID controllers tuning and process identification, also it returns the close-loop simulations when the process incorporates delay/dead-time.

## Developer enviroment

### Install

```bash
virtualenv venv
source venv/bin/activate

pip install -r requirements.txt
pip install -r requirements-dev.txt
```

### Run tests

```bash
## Alfaro123c an FOM validation tests
pytest test/test.py
```
