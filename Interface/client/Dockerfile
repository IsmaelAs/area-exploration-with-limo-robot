FROM node:18.13

RUN npm i -g @angular/cli

COPY . ./home/client
WORKDIR /home/client

RUN npm install

CMD ["npm", "start"]