using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using NUnit.Framework;
using Unity.Physics.Authoring;
using UnityEditor.PackageManager;
using UnityEngine;

namespace Unity.Physics.Tests.Authoring
{
    class HelpURLs_UnitTests
    {
        [Test]
        public void ConstPackageVersion_MatchesActualPackageVersion()
        {
            var packageInfo = PackageInfo.FindForAssetPath("Packages/com.unity.physics/package.json");
            var expectedPackageVersionMajorMinor = new Regex(@"\d+\.\d+(?=\.\d+)").Match(packageInfo.version).Value;
            Assume.That(expectedPackageVersionMajorMinor, Is.Not.Empty, "Unable to determine current package version.");

            Assert.That(
                HelpURLs.PackageVersion, Is.EqualTo(expectedPackageVersionMajorMinor),
                $"Update {nameof(HelpURLs)}.{nameof(HelpURLs.PackageVersion)}"
            );
        }

        static readonly IEnumerable<Type> AllAuthoringTypes = typeof(PhysicsShapeAuthoring).Assembly.GetTypes().Where(
            t => t.IsPublic
            && !t.IsAbstract
            && !t.IsGenericType
            && (t.IsSubclassOf(typeof(ScriptableObject)) || t.IsSubclassOf(typeof(MonoBehaviour)))
            && t.GetCustomAttributes().Count(a => a is ObsoleteAttribute) == 0
        );

        [Test]
        public void AuthoringType_HasProperlyFormattedHelpURL([ValueSource(nameof(AllAuthoringTypes))] Type type)
        {
            var attr = type.GetCustomAttribute(typeof(HelpURLAttribute)) as HelpURLAttribute;
            Assume.That(attr, Is.Not.Null, "Public authoring type has no HelpURLAttribute");

            var url = attr.URL;
            Assert.That(url, Contains.Substring(type.FullName), "HelpURLAttribute does not reference proper type name");
        }
    }
}
