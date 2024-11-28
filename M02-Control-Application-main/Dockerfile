FROM python:3.10-slim-bookworm
LABEL maintainer "Lucas Vogt | TUD"

WORKDIR /usr/local/bin
RUN mkdir -p /home/Control
COPY requirements.txt .
COPY Control_Application.py /home/Control/

#RUN apt-get install libprotobuf-dev 
RUN pip install --upgrade pip
RUN pip install --no-cache-dir -r requirements.txt
EXPOSE 4840
CMD ["python3", "/home/Control/Control_Application.py"]
