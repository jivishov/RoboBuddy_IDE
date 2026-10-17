import json
import threading
import unittest
import urllib.error
import urllib.request
from hardware_bridge.server import BridgeServer

class HTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server=BridgeServer(port=0,demo=True,token="test-only-demo-token")
        cls.thread=threading.Thread(target=cls.server.serve_forever,daemon=True);cls.thread.start()
        cls.base=f"http://127.0.0.1:{cls.server.server_port}"
    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown();cls.server.server_close()
    def request(self,path="/api/state",data=None,headers=None):
        hdr={"Origin":self.base,"Authorization":"Bearer test-only-demo-token",**(headers or {})}
        if data is not None:hdr["Content-Type"]="application/json"
        req=urllib.request.Request(self.base+path,data=json.dumps(data).encode() if data is not None else None,headers=hdr)
        try:
            with urllib.request.urlopen(req,timeout=2) as r:return r.status,dict(r.headers),r.read()
        except urllib.error.HTTPError as e:return e.code,dict(e.headers),e.read()
    def test_health_does_not_connect_any_bus(self):
        status,_,body=self.request("/api/health",headers={"Authorization":""})
        self.assertEqual(status,200);self.assertEqual(json.loads(body)["mode"],"demo")
        self.assertIsNone(self.server.controller.bus)
    def test_missing_token_is_rejected(self):
        status,_,_=self.request(headers={"Authorization":""});self.assertEqual(status,401)
    def test_unapproved_origin_is_rejected_even_with_valid_token(self):
        status,headers,_=self.request(headers={"Origin":"https://evil.example"})
        self.assertEqual(status,403);self.assertNotIn("Access-Control-Allow-Origin",headers)
    def test_dns_rebinding_host_is_rejected(self):
        status,_,_=self.request(headers={"Host":"evil.example"});self.assertEqual(status,403)
    def test_bootstrap_is_not_available_to_the_hosted_origin(self):
        status,_,body=self.request("/api/bootstrap",headers={"Origin":"https://robobuddy.dev","Sec-Fetch-Site":"cross-site"})
        self.assertEqual(status,403);self.assertNotIn(b"test-only-demo-token",body)
    def test_local_page_can_bootstrap_without_authentication(self):
        status,_,body=self.request("/api/bootstrap",headers={"Authorization":"","Sec-Fetch-Site":"same-origin"})
        self.assertEqual(status,200);self.assertEqual(json.loads(body)["token"],"test-only-demo-token")
    def test_private_and_python_files_are_not_served(self):
        for path in ("/.git/config","/hardware_bridge/server.py","/../AGENTS.md","/hardware_bridge/requirements.txt"):
            with self.subTest(path=path):self.assertEqual(self.request(path)[0],404)
    def test_page_is_served_with_no_store_and_no_referrer(self):
        status,headers,body=self.request("/hardware.html")
        self.assertEqual(status,200);self.assertEqual(headers["Cache-Control"],"no-store")
        self.assertEqual(headers["Referrer-Policy"],"no-referrer")
        self.assertNotIn(b"test-only-demo-token",body)
    def test_unknown_operations_and_nonobject_payloads_are_rejected(self):
        self.assertEqual(self.request("/api/nope",{})[0],404)
        self.assertEqual(self.request("/api/session",[])[0],400)
    def test_json_body_limit_is_enforced(self):
        self.assertEqual(self.request("/api/session",{"a":"x"*33000})[0],413)
    def test_mutations_require_origin(self):
        req=urllib.request.Request(self.base+"/api/session",data=b"{}",headers={"Authorization":"Bearer test-only-demo-token","Content-Type":"application/json"})
        with self.assertRaises(urllib.error.HTTPError) as context:urllib.request.urlopen(req,timeout=2)
        self.assertEqual(context.exception.code,403)

if __name__=="__main__": unittest.main()

