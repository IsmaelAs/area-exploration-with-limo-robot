const fs = require('fs');
const targetPath = './src/environments/environment.ts';


const envConfigFile = `export const environment = {
  BACKEND_IP: '${process.env["BACKEND_IP"]}',
};`;

console.log('The content of `environment.ts` will be: \n');
console.log(envConfigFile);

fs.writeFile(targetPath, envConfigFile, function (err: Error) {
  if (err) {
    throw console.error(err);
  } else {
    console.log(`environment.ts file is created`);
  }
});