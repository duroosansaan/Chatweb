const ACCESS_TOKEN = process.env.META_ACCESS_TOKEN;
const API_VERSION = 'v18.0';

export async function sendPrivateMessage(recipientId, text) {
  const url = `https://graph.facebook.com/${API_VERSION}/me/messages?access_token=${ACCESS_TOKEN}`;
  
  await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      recipient: { id: recipientId },
      message: { text: text },
    }),
  });
}
