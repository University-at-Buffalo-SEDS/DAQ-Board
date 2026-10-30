import pathlib
import unittest
import subprocess
import tempfile
import re


ROOT = pathlib.Path(__file__).resolve().parents[1]


class SedsnetRoutingContract(unittest.TestCase):
    def test_local_acquisition_streams_have_a_can_route_override(self):
        sources = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (ROOT / "Core" / "Src").glob("*.c")
        )
        self.assertNotIn("seds_router_set_route", sources)
        self.assertEqual(sources.count("seds_router_set_typed_route("), 6)
        self.assertIn("seds_router_set_typed_route(r, -1, (uint32_t)SEDS_DT_KG1000,\n"
                      "                                       g_can_side_id, true)", sources)
        self.assertIn("seds_router_set_typed_route(r, -1, (uint32_t)SEDS_DT_KG50,", sources)
        self.assertIn("seds_router_set_typed_route(r, -1, (uint32_t)SEDS_DT_DAQ_KG50_SELECTED,", sources)

    def test_combined_pressure_uses_the_same_can_fallback_as_scalar_loadcell(self):
        source = (ROOT / "Core/Src/telemetry.c").read_text()
        start = source.index("  result = seds_router_set_typed_route(r, -1,")
        end = source.index("  if (result != SEDS_OK)", start)
        body = source[start:end]
        names = sorted(set(re.findall(r"SEDS_DT_\w+", body)))
        defines = "\n".join(f"#define {name} {i+1}U" for i, name in enumerate(names))
        code = r"""
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#define SEDS_OK 0
static unsigned routed[32];
static int seds_router_set_typed_route(void *r,int ingress,uint32_t type,int side,bool enabled) {
 assert(r && ingress==-1 && side==3 && enabled);routed[type]++;return SEDS_OK;
}
""" + defines + "\nint main(void) {void *r=(void*)1;int g_can_side_id=3,result=0;\n" + body + r"""
 assert(result==SEDS_OK);
 assert(routed[SEDS_DT_DAQ_LOADCELL_CHAMBER]==1);
 assert(routed[SEDS_DT_KG1000]==1 && routed[SEDS_DT_KG50]==1);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            exe=str(pathlib.Path(tmp)/"routes")
            subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror","-x","c","-","-o",exe],input=code,text=True,check=True)
            subprocess.run([exe],check=True)


if __name__ == "__main__":
    unittest.main()
