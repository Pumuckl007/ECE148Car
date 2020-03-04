const http = require('http');

const requestListener = function (req, res) {
  res.writeHead(200);
  res.end('IP Saved');
  console.log(req.method);
  req.on('data', function (data) {
            console.log(data.toString());
        });
}

const server = http.createServer(requestListener);
server.listen(8080);
