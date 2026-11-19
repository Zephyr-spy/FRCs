import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from check_config import warnings

class ConfigurationWarningTest(unittest.TestCase):
    def test_unknowns_are_clickable_and_clear_after_edit(self):
        source = '''@Pending("安裝角度，度")
public static final double yawDegrees = Double.NaN;
@Pending("重量 kg")
public static final double massKg = Double.NaN;
@Pending("名稱")
public static final String tableName = "";
@Pending("校正確認")
public static final boolean confirmed = false;'''
        result = warnings(source)
        self.assertEqual([x[0] for x in result], [2,4,6,8])
        fixed = source.replace('yawDegrees = Double.NaN','yawDegrees = 0').replace('massKg = Double.NaN','massKg = 50').replace('= ""','= "limelight-test"').replace('= false','= true')
        self.assertEqual(warnings(fixed), [])
    def test_zero_is_not_valid_mass_but_is_valid_mount_angle(self):
        self.assertEqual(len(warnings('@Pending("重量") public static final double massKg = 0;')),1)
        self.assertEqual(warnings('@Pending("角度") public static final double yawDegrees = 0;'),[])
    def test_malformed_annotation_is_not_silently_skipped(self):
        self.assertTrue(warnings('@Pending("錯誤") final Object value = null;'))

if __name__ == '__main__': unittest.main()
