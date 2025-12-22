import { useEffect } from "react";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";

export default function Home(): null {
  const { siteConfig } = useDocusaurusContext();

  useEffect(() => {
    const targetUrl = `${siteConfig.baseUrl}docs`.replace(/\/+/g, "/");
    window.location.replace(targetUrl);
  }, [siteConfig.baseUrl]);

  return null;
}
