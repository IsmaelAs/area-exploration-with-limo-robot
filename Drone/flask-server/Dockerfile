FROM python:3.8

WORKDIR /home/flask-server

RUN python3 -m venv venv 
RUN . venv/bin/activate

COPY . /home/flask-server

RUN pip install -r requirements.txt
CMD ["flask", "--app", "main", "run"]





